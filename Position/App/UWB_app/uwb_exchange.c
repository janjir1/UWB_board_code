#include "cmsis_os.h"
#include "cmsis_os2.h"
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>

#include "main.h"
#include "../Generic/my_print.h"
#include "DWM3000_setup.h"
#include "DWM3000_driver.h"
#include "messages.h"
#include "../UWB_app/uwb_network.h"
#include "uwb_exchange.h"
#include "distance.h"
#include "lsm6dsv.h"
#include "calibrate.h"

/* -----------------------------------------------------------------------
 * Forward declarations
 * ----------------------------------------------------------------------- */
static bool uwb_send_sync_reply(const msg_t *sync_rx);
static bool uwb_sync_to_all_id(uint8_t seq_num);
static bool uwb_send_POLL    (uint8_t seq_num, uint16_t target_id, uint64_t *out_poll_tx);
static bool uwb_send_RESPONSE(uint8_t seq_num, uint16_t target_id, uint64_t *out_resp_tx);
static bool uwb_send_FINAL   (uint8_t seq_num, uint16_t target_id, msg_final_t *final_msg);

static void remove_silent_passive_peers(const uint16_t *expected_ids,
                                        uint8_t expected_count,
                                        const uint16_t *heard_ids,
                                        uint8_t heard_count);

static uwb_etwr_result_t uwb_wait_for_PASSIVES(uint64_t entries[],
                                               int16_t entry_pwr_diff_q8[],
                                               bool antenna_unreliable[],
                                               uint16_t entry_ids[],
                                               uint8_t *out_count,
                                               msg_passive_t out_passive_msgs[],
                                               uint16_t target_id);

static uwb_etwr_result_t uwb_send_PASSIVE(uint16_t initiator_id,
                                          uint16_t responder_id,
                                          msg_passive_t *passive_msg);

/* -----------------------------------------------------------------------
 * Internal macros
 * ----------------------------------------------------------------------- */

/** @brief Construct a @c uwb_rx_meas_t inline from a @c dwm_rx_frame_t. */
#define RX_MEAS_FROM_FRAME(frame) \
    (uwb_rx_meas_t){ .ts = (frame).rx_timestamp, \
                     .pwr_diff_q8 = (frame).rssi_q8 - (frame).fp_q8 }

/** @brief Extract high 32 bits of a uint64_t for two-word mprintf. */
#define U64_HI(x) ((uint32_t)((x) >> 32))

/** @brief Extract low 32 bits of a uint64_t for two-word mprintf. */
#define U64_LO(x) ((uint32_t)((x) & 0xFFFFFFFFU))

/**
 * @brief Print a 40-bit UWB timestamp label/value pair.
 *
 * Uses two @c %08lX fields because mprintf does not support @c %llX.
 */
#define MPRINT_TS(label, val) \
    mprintf("UWB_eTWR_TEST - %-20s: 0x%08lX%08lX\r\n", \
            (label), U64_HI(val), U64_LO(val))

/* -----------------------------------------------------------------------
 * Message logging helpers
 * ----------------------------------------------------------------------- */

/**
 * @brief Return a short 3-letter string label for a message type.
 *
 * @param t  Raw message type byte.
 * @return   Constant string label, or @c "???" for unknown types.
 */
static const char *msg_type_str(uint8_t t)
{
    switch (t) {
        case MSG_TYPE_SYNC:     return "SYN";
        case MSG_TYPE_POLL:     return "POL";
        case MSG_TYPE_RESPONSE: return "RSP";
        case MSG_TYPE_FINAL:    return "FIN";
        case MSG_TYPE_PASSIVE:  return "PAS";
        case MSG_TYPE_SHARE:    return "SHR";
        default:                return "???";
    }
}

/**
 * @brief Decode a frame and log the decoded message type and address fields.
 *
 * @param[in]  f  Raw RX frame to decode.
 * @param[out] m  Populated @c msg_t on return.
 */
static inline void msg_decode_log(const dwm_rx_frame_t *f, msg_t *m)
{
    msg_decode(f, m);
    mprintf(">%s %04X->%04X\r\n", msg_type_str(m->type), m->sender, m->receiver);
}

/**
 * @brief Encode a message, log it, and return the TX frame.
 *
 * @param[in] m  Message to encode.
 * @return       Ready-to-transmit @c dwm_tx_frame_t.
 */
static inline dwm_tx_frame_t msg_encode_log(const msg_t *m)
{
    mprintf("<%s %04X->%04X\r\n", msg_type_str(m->type), m->sender, m->receiver);
    return msg_encode(m);
}

/* -----------------------------------------------------------------------
 * Internal helpers
 * ----------------------------------------------------------------------- */

/**
 * @brief Sub-microsecond busy-wait using a hardware timer.
 *
 * @param us  Delay in microseconds.
 */
static void delay_us(uint32_t us)
{
    __HAL_TIM_SET_COUNTER(&htim2, 0);
    while (__HAL_TIM_GET_COUNTER(&htim2) < us);
}

/**
 * @brief Staggered TX delay derived from device short address.
 *
 * Spreads devices uniformly across a 0–10000 µs window based on the
 * low byte of their address, reducing simultaneous transmission collisions.
 * Uses osDelay() for full milliseconds and delay_us() for the remainder.
 *
 * @param short_addr  This device's 16-bit network address.
 */
static void uwb_id_delay(uint16_t short_addr)
{
    uint8_t  low      = (uint8_t)(short_addr & 0xFF);
    uint32_t total_us = ((uint32_t)low * 10000U) / 255U;
    uint32_t ms       = total_us / 1000U;
    uint32_t us       = total_us % 1000U;

    if (ms > 0) osDelay(ms);
    if (us > 0) delay_us(us);
}

/**
 * @brief Check whether a given ID appears in a SYNC message's peer list.
 *
 * @param sync  Pointer to the received SYNC payload.
 * @param id    Device ID to search for.
 * @return true if @p id is found, false otherwise.
 */
static bool sync_peer_contains(const msg_sync_t *sync, uint16_t id)
{
    for (int i = 0; i < sync->peer_count; i++)
        if (sync->peer_ids[i] == id) return true;
    return false;
}

/**
 * @brief Pad the current iteration to at least @c T_SYNC_RX_ANSWER ms from @p t_start.
 *
 * Ensures all devices in the SYNC window finish at approximately the same time
 * regardless of how quickly they processed the SYNC reply.
 */
#define SYNC_PAD(t_start) \
    do { \
        uint32_t _e = osKernelGetTickCount() - (t_start); \
        if (_e < T_SYNC_RX_ANSWER) osDelay(T_SYNC_RX_ANSWER - _e); \
    } while (0)

/**
 * @brief Validate that the received sequence number matches the expected value.
 *
 * Logs an error if the numbers differ.
 *
 * @param rx_seq  Sequence number extracted from the received message.
 * @param tag     Short label printed in the error message.
 * @return true if the sequence number is correct, false otherwise.
 */
static bool seq_ok(uint8_t rx_seq, const char *tag)
{
    uint8_t exp = network_get_expected_seq_num();
    if (rx_seq != exp) {
        mprintf("ERROR: bad seq [%s] rx=%d exp=%d\r\n", tag, rx_seq, exp);
        return false;
    }
    return true;
}

/**
 * @brief Handle an unexpected message received during an exchange phase.
 *
 * Infers the current master from the message fields and delays for
 * @c DEEP_SLEEP to allow the ongoing exchange to complete before
 * this node re-enters the ranging loop. Passive and unknown message
 * types are silently ignored.
 *
 * @param rx_msg  The unexpected message that was received.
 * @return Always returns true.
 */
bool uwb_unexpected_message_handler(const msg_t *rx_msg)
{
    mprintf("uwb_unexpected_message_handler 0x%02X\r\n", rx_msg->type);
    switch (rx_msg->type) {

        case MSG_TYPE_POLL:
        case MSG_TYPE_FINAL:
            network_set_master(rx_msg->receiver);
            osDelay(DEEP_SLEEP - 20);
            break;

        case MSG_TYPE_RESPONSE:
        case MSG_TYPE_SHARE:
            network_set_master(rx_msg->sender);
            osDelay(DEEP_SLEEP - 20);
            break;

        case MSG_TYPE_SYNC:
            if (rx_msg->receiver != ALL_ID) {
                network_set_master(rx_msg->sender);
                osDelay(DEEP_SLEEP - 20);
            }
            break;

        case MSG_TYPE_PASSIVE:
        default:
            break;
    }
    return true;
}

/* -----------------------------------------------------------------------
 * Public API — uwb_sync
 * ----------------------------------------------------------------------- */

/**
 * @brief Perform one UWB synchronisation iteration.
 *
 * The master broadcasts a SYNC frame and listens for join replies.
 * Slaves listen for a SYNC broadcast; if their ID is not yet in the
 * master's peer list they reply with their own SYNC to request admission.
 * A broadcast-storm guard demotes the master if SYNCs are sent faster
 * than every 50 ms with no replies. Slaves promote themselves to master
 * after @c SYNC_TIMEOUT_MAX consecutive timeouts.
 *
 * @return Outcome code indicating role and readiness for the TWR phase.
 */
uwb_sync_result_t uwb_sync(void)
{
    dwt_writesysstatuslo(DWT_INT_RXFTO_BIT_MASK | DWT_INT_RXPTO_BIT_MASK);
    dwm_rx_flush();

    if (network_is_master()) {

        uint8_t seq_num = network_get_expected_seq_num() + 1;
        network_set_expected_seq_num(seq_num);
        osDelay(2);

        /* Demote self to avoid SYNC broadcast storms between multiple masters. */
        static uint32_t last_sync_time  = 0;
        static uint8_t  rapid_sync_count = 0;
        uint32_t now = osKernelGetTickCount();

        if ((now - last_sync_time) < 50U) {
            rapid_sync_count++;
            if (rapid_sync_count > 5) {
                mprintf("ERROR: SYNC broadcast storm detected! Demoting self.\r\n");
                rapid_sync_count = 0;
                network_set_master(0);
                return UWB_SYNC_NEW_SLAVE;
            }
        } else {
            rapid_sync_count = 0;
        }
        last_sync_time = now;

        if (network_get_count() == 0)
            uwb_id_delay(network_get_ownid());

        if (!uwb_sync_to_all_id(seq_num)) {
            mprintf("ERROR: SYNC TX failed\r\n");
            return UWB_SYNC_TX_FAILED;
        }

        osThreadFlagsSet(AccelerometerHandle, 0x01);

        uint32_t t_start = osKernelGetTickCount();
        dwm_rx_frame_t rx_frame = {0};
        bool first_rx = true;
        while (1) {
            uint32_t elapsed = osKernelGetTickCount() - t_start;
            if (elapsed >= T_SYNC_RX_ANSWER) break;

            dwm_rx(&rx_frame, T_SYNC_RX_ANSWER - elapsed, true, first_rx, NULL);
            first_rx = (rx_frame.type != DWM_RX_OK);

            switch (rx_frame.type) {
                case DWM_RX_OK: {
                    msg_t rx_msg;
                    msg_decode_log(&rx_frame, &rx_msg);

                    if (rx_msg.type != MSG_TYPE_SYNC) {
                        mprintf("ERROR: unexpected msg 0x%02X in SYNC window\r\n", rx_msg.type);
                        break;
                    }

                    if (rx_msg.receiver == network_get_ownid()) {
                        network_add_peer(rx_msg.sender);
                        mprintf("[SYNC] peer 0x%04X joined\r\n", rx_msg.sender);
                        SYNC_PAD(t_start);
                        return UWB_SYNC_MASTER;
                    } else {
                        /* Dual-master conflict — higher ID wins. */
                        if (rx_msg.sender > network_get_ownid()) {
                            network_set_master(rx_msg.sender);
                            mprintf("[SYNC] dual master — yielding to 0x%04X\r\n", rx_msg.sender);
                            SYNC_PAD(t_start);
                            return UWB_SYNC_NEW_SLAVE;
                        }
                    }
                    break;
                }
                case DWM_RX_ERR:
                    mprintf("ERROR: SYNC RX error 0x%08lX\r\n", rx_frame.status);
                    break;
                case DWM_RX_TIMEOUT:
                    break;
            }
        }

        return UWB_SYNC_MASTER_NO_REPLY;

    } else {

        dwm_rx_frame_t rx_frame = {0};
        uint8_t  timeout_count = 0;
        int16_t  clock_offset;
        bool     first_rx = true;
        dwm_rx_flush();

        while (1) {
            osDelay(1);
            dwm_rx(&rx_frame, T_SYNC_RX_SET, true, first_rx, &clock_offset);
            first_rx = (rx_frame.type != DWM_RX_OK);

            switch (rx_frame.type) {
                case DWM_RX_OK: {
                    msg_t rx_msg;
                    msg_decode_log(&rx_frame, &rx_msg);

                    switch (rx_msg.type) {

                        case MSG_TYPE_SYNC: {
                            if (rx_msg.receiver == ALL_ID) {
                                uint32_t t_start = osKernelGetTickCount();
                                osThreadFlagsSet(AccelerometerHandle, 0x01);
                                calibrate_set_clock_offset_sync(clock_offset);

                                if (rx_msg.sender != network_get_master()) {
                                    network_set_master(rx_msg.sender);
                                } else {
                                    network_update_peers_from_sync(rx_msg.sender,
                                                                   rx_msg.data.sync.peer_ids,
                                                                   rx_msg.data.sync.peer_count);
                                    network_set_expected_seq_num(rx_msg.data.sync.seq_num);
                                }

                                if (sync_peer_contains(&rx_msg.data.sync, network_get_ownid())) {
                                    mprintf("[SYNC] ack by master 0x%04X\r\n", rx_msg.sender);
                                    network_set_acknowledged(true);
                                    SYNC_PAD(t_start);
                                    return UWB_SYNC_SLAVE_ACKNOWLEDGED;
                                } else {
                                    network_set_acknowledged(false);
                                    if (!uwb_send_sync_reply(&rx_msg)) {
                                        SYNC_PAD(t_start);
                                        return UWB_SYNC_SLAVE_REPLY_FAILED;
                                    }
                                    SYNC_PAD(t_start);
                                    return UWB_SYNC_SLAVE_PENDING;
                                }
                            }
                            break;
                        }

                        case MSG_TYPE_POLL:
                        case MSG_TYPE_FINAL:
                            network_set_master(rx_msg.receiver);
                            osDelay(DEEP_SLEEP - 10);
                            return UWB_SYNC_UNEXPECTED_MASTER;

                        case MSG_TYPE_RESPONSE:
                        case MSG_TYPE_SHARE:
                            network_set_master(rx_msg.sender);
                            osDelay(DEEP_SLEEP - 10);
                            return UWB_SYNC_UNEXPECTED_MASTER;

                        case MSG_TYPE_PASSIVE:
                            return UWB_SYNC_UNEXPECTED_MASTER;

                        default:
                            break;
                    }
                    break;
                }

                case DWM_RX_ERR:
                    mprintf("ERROR: SYNC RX error 0x%08lX\r\n", rx_frame.status);
                    break;

                case DWM_RX_TIMEOUT:
                    timeout_count++;
                    uwb_id_delay(network_get_ownid());

                    if (timeout_count >= SYNC_TIMEOUT_MAX) {
                        network_set_master(network_get_ownid());
                        mprintf("[SYNC] no master found — promoted to master\r\n");
                        return UWB_SYNC_NEW_MASTER;
                    }
                    break;
            }
        }
    }
}

/* -----------------------------------------------------------------------
 * Static TWR helpers
 * ----------------------------------------------------------------------- */

/**
 * @brief Wait for a RESPONSE frame from a specific responder.
 *
 * Blocks until a valid RESPONSE is received from @p target_id addressed
 * to the master, or until @c T_RESPONSE_RX_WAIT elapses. On timeout the
 * peer is removed from the network. Unexpected messages are forwarded to
 * @ref uwb_unexpected_message_handler.
 *
 * @param[in]  target_id                  Expected sender of the RESPONSE.
 * @param[out] out_resp_rx_ts             Calibrated RX timestamp of the RESPONSE.
 * @param[out] out_resp_pwr_diff_q8       RSSI minus first-path power (Q8).
 * @param[out] out_resp_antenna_unreliable  true if antenna quality check failed.
 * @return true on success, false on timeout or TX error.
 */
static bool uwb_wait_for_RESPONSE(uint16_t  target_id,
                                   uint64_t *out_resp_rx_ts,
                                   int16_t  *out_resp_pwr_diff_q8,
                                   bool     *out_resp_antenna_unreliable)
{
    dwm_rx_frame_t rx_frame = {0};
    uint32_t t_start = osKernelGetTickCount();
    bool first_rx = true;

    while (1) {
        uint32_t elapsed = osKernelGetTickCount() - t_start;
        if (elapsed >= T_RESPONSE_RX_WAIT) {
            network_remove_peer(target_id);
            mprintf("ERROR: RESPONSE timeout — removed 0x%04X\r\n", target_id);
            return false;
        }

        dwm_rx(&rx_frame, T_RESPONSE_RX_WAIT - elapsed, true, first_rx, NULL);
        first_rx = (rx_frame.type != DWM_RX_OK);

        switch (rx_frame.type) {
            case DWM_RX_OK: {
                msg_t rx_msg;
                msg_decode_log(&rx_frame, &rx_msg);

                if (rx_msg.type     == MSG_TYPE_RESPONSE &&
                    rx_msg.receiver == network_get_master() &&
                    rx_msg.sender   == target_id) {

                    if (!seq_ok(rx_msg.data.response.seq_num, "RESP"))
                        break;

                    *out_resp_rx_ts = calibrate_rx_timestamp(rx_frame.rx_timestamp,
                                                             rx_frame.rssi_q8,
                                                             rx_msg.sender,
                                                             out_resp_antenna_unreliable);
                    *out_resp_pwr_diff_q8 = rx_frame.rssi_q8 - rx_frame.fp_q8;
                    return true;
                }
                if (uwb_unexpected_message_handler(&rx_msg)) {
                    return UWB_TWR_UNEXPECTED_MASTER;
                }
                break;
            }
            case DWM_RX_ERR:
                mprintf("ERROR: RESPONSE RX error 0x%08lX\r\n", rx_frame.status);
                break;
            case DWM_RX_TIMEOUT:
                network_remove_peer(target_id);
                mprintf("ERROR: RESPONSE timeout — removed 0x%04X\r\n", target_id);
                return false;
        }
    }
}

/**
 * @brief Wait for a FINAL frame addressed to this node from the master.
 *
 * Blocks until a valid FINAL is received or @c T_FINAL_RX_WAIT elapses.
 * On success the FINAL data is stored via @ref network_store_final.
 *
 * @return true on success, false on timeout or RX error.
 */
static bool uwb_wait_for_FINAL(void)
{
    dwm_rx_frame_t rx_frame = {0};
    uint32_t t_start = osKernelGetTickCount();
    bool first_rx = true;

    while (1) {
        uint32_t elapsed = osKernelGetTickCount() - t_start;
        if (elapsed >= T_FINAL_RX_WAIT) {
            mprintf("ERROR: FINAL timeout\r\n");
            return false;
        }

        dwm_rx(&rx_frame, T_FINAL_RX_WAIT - elapsed, true, first_rx, NULL);
        first_rx = (rx_frame.type != DWM_RX_OK);

        switch (rx_frame.type) {
            case DWM_RX_OK: {
                msg_t rx_msg;
                msg_decode_log(&rx_frame, &rx_msg);

                if (rx_msg.type     == MSG_TYPE_FINAL &&
                    rx_msg.receiver == network_get_ownid() &&
                    rx_msg.sender   == network_get_master()) {

                    if (!seq_ok(rx_msg.data.final.seq_num, "FINAL"))
                        break;

                    bool antenna_unreliable = false;
                    uwb_rx_meas_t final_meas = {
                        .ts = calibrate_rx_timestamp(rx_frame.rx_timestamp,
                                                     rx_frame.rssi_q8,
                                                     rx_msg.sender,
                                                     &antenna_unreliable),
                        .pwr_diff_q8 = rx_frame.rssi_q8 - rx_frame.fp_q8
                    };

                    network_store_final(&rx_msg.data.final, &final_meas, antenna_unreliable);
                    return true;
                }
                mprintf("ERROR: unexpected msg 0x%02X from 0x%04X during FINAL wait\r\n",
                        rx_msg.type, rx_msg.sender);
                break;
            }
            case DWM_RX_ERR:
                mprintf("ERROR: FINAL RX error 0x%08lX\r\n", rx_frame.status);
                break;
            case DWM_RX_TIMEOUT:
                mprintf("ERROR: FINAL timeout\r\n");
                return false;
        }
    }
}

/* -----------------------------------------------------------------------
 * Public API — uwb_extended_twr
 * ----------------------------------------------------------------------- */

/**
 * @brief Execute the extended TWR ranging exchange for one round.
 *
 * Dispatches on @p sync_result to run the appropriate role:
 * - **Master**: selects the highest-uncertainty target, sends POLL,
 *   waits for RESPONSE and PASSIVE frames, then sends FINAL.
 * - **Slave / Responder**: waits for a POLL addressed to self, sends
 *   RESPONSE, collects PASSIVE frames, then waits for FINAL.
 * - **Slave / Passive observer**: records POLL and RESPONSE timestamps
 *   and transmits a PASSIVE frame to the master.
 * - **Invalid sync states**: returns @c UWB_TWR_NOT_ENOUGH_DEVICES immediately.
 *
 * IMU data is consumed at entry via @c osThreadFlagsWait and stored in the
 * network state for inclusion in the FINAL/PASSIVE messages.
 *
 * @param sync_result  Role and peer information from the preceding @ref uwb_sync call.
 * @return Exchange outcome code.
 */
uwb_etwr_result_t uwb_extended_twr(uwb_sync_result_t sync_result)
{
    network_reset_measurements();
    dwm_rx_flush();
    osDelay(1);

    uint32_t flags = osThreadFlagsWait(0x02, osFlagsWaitAll, 0);
    float pitch_rad = 0, speed_horiz = 0, vel_z = 0;
    if (!(flags & 0x80000000U) && (flags & 0x02))
        imu_get_results(&pitch_rad, &speed_horiz, &vel_z);
    mprintf("IMU data read");
    calibrate_set_pitch(pitch_rad);

    switch (sync_result) {

        /* ---- MASTER: initiates DS-TWR exchange ---- */
        case UWB_SYNC_MASTER:
        case UWB_SYNC_MASTER_NO_REPLY:
        {
            if (network_get_count() < 1) {
                mprintf("ERROR: no peers — skipping TWR\r\n");
                return UWB_TWR_NOT_ENOUGH_DEVICES;
            }

            mprintf("[TWR] M peers=%d\r\n", network_get_count());
            uint16_t target_id = network_get_highest_uncertainty();
            if (target_id == 0) {
                mprintf("ERROR: no valid target — skipping TWR\n");
                return UWB_TWR_NOT_ENOUGH_DEVICES;
            }
            mprintf("[CERT] selected target=0x%04X\n", target_id);

            osDelay(2);

            uint64_t poll_tx;
            if (!uwb_send_POLL(network_get_expected_seq_num(), target_id, &poll_tx)) {
                mprintf("ERROR: POLL TX failed\r\n");
                return UWB_TWR_TX_FAILED;
            }

            mprintf("[TWR] MASTER - 0x%04X\r\n", target_id);

            msg_final_t final_msg = {
                .seq_num    = network_get_expected_seq_num(),
                .poll_tx_ts = calibrate_tx_timestamp(poll_tx),
            };
            final_msg.IMU_vel_horiz = speed_horiz;
            final_msg.IMU_vel_vert  = vel_z;

            if (!uwb_wait_for_RESPONSE(target_id,
                                       &final_msg.resp_rx_ts,
                                       &final_msg.resp_pwr_diff_q8,
                                       &final_msg.resp_antenna_unreliable))
                return UWB_TWR_TIMEOUT;

            msg_passive_t passive_msgs[NETWORK_MAX_PEERS - 2];
            uwb_wait_for_PASSIVES(final_msg.entries,
                                  final_msg.entry_pwr_diff_q8,
                                  final_msg.entry_antenna_unreliable,
                                  final_msg.entry_id,
                                  &final_msg.entry_count,
                                  passive_msgs,
                                  target_id);

            mprintf("[TWR] M pass=%d\r\n", final_msg.entry_count);

            if (!uwb_send_FINAL(network_get_expected_seq_num(), target_id, &final_msg))
                return UWB_TWR_TX_FAILED;

            return UWB_TWR_EXCHANGE_COMPLETE;
        }

        /* ---- SLAVE: respond to or passively observe the DS-TWR exchange ---- */
        case UWB_SYNC_SLAVE_ACKNOWLEDGED:
        case UWB_SYNC_SLAVE_PENDING:
        {
            if (network_get_count() < 1) {
                mprintf("ERROR: no peers — skipping TWR\r\n");
                return UWB_TWR_NOT_ENOUGH_DEVICES;
            }

            dwm_rx_frame_t rx_frame = {0};
            uint32_t t_start = osKernelGetTickCount();
            int16_t  clock_offset;
            bool     first_rx = true;

            while (1) {
                uint32_t elapsed = osKernelGetTickCount() - t_start;
                if (elapsed >= T_TWR_RX_WAIT) {
                    mprintf("ERROR: TWR window expired\r\n");
                    return UWB_TWR_TIMEOUT;
                }

                dwm_rx(&rx_frame, T_TWR_RX_WAIT - elapsed, true, first_rx, &clock_offset);
                first_rx = (rx_frame.type != DWM_RX_OK);

                switch (rx_frame.type) {
                    case DWM_RX_OK: {
                        msg_t rx_msg;
                        msg_decode_log(&rx_frame, &rx_msg);

                        if (rx_msg.type == MSG_TYPE_POLL) {

                            calibrate_set_clock_offset_poll(clock_offset);

                            if (rx_msg.sender != network_get_master()) {
                                mprintf("ERROR: POLL from unexpected sender 0x%04X (master: 0x%04X)\r\n",
                                        rx_msg.sender, network_get_master());
                                network_set_master(rx_msg.receiver);
                                osDelay(DEEP_SLEEP - 10);
                                return UWB_TWR_UNEXPECTED_MASTER;
                            }

                            if (!seq_ok(rx_msg.data.poll.seq_num, "POLL"))
                                return UWB_TWR_TIMEOUT;

                            if (rx_msg.receiver == network_get_ownid()) {
                                /* ---- Responder path ---- */
                                delay_us(50);

                                network_t *net = network_get_network();
                                net->self.imu_vel_vert  = vel_vert_to_u8(vel_z);
                                net->self.imu_vel_horiz = vel_horiz_to_u8(speed_horiz);

                                bool antenna_unreliable = false;
                                uwb_rx_meas_t poll_meas = {
                                    .ts = calibrate_rx_timestamp(rx_frame.rx_timestamp,
                                                                  rx_frame.rssi_q8,
                                                                  rx_msg.sender,
                                                                  &antenna_unreliable),
                                    .pwr_diff_q8 = rx_frame.rssi_q8 - rx_frame.fp_q8
                                };
                                network_store_poll(&rx_msg.data.poll, &poll_meas, antenna_unreliable);

                                uint64_t resp_tx;
                                if (!uwb_send_RESPONSE(network_get_expected_seq_num(),
                                                       network_get_master(), &resp_tx)) {
                                    mprintf("ERROR: RESPONSE TX failed\r\n");
                                    return UWB_TWR_TX_FAILED;
                                }

                                mprintf("[TWR] RESPONDER - 0x%04X\r\n", rx_msg.sender);
                                network_store_resp_tx(calibrate_tx_timestamp(resp_tx));

                                uint64_t entries[NETWORK_MAX_PEERS - 2];
                                int16_t  entry_pwr_diff_q8[NETWORK_MAX_PEERS - 2];
                                bool     entry_antenna_unreliable[NETWORK_MAX_PEERS - 2];
                                uint16_t entry_ids[NETWORK_MAX_PEERS - 2];
                                uint8_t  count = 0;
                                msg_passive_t passive_msgs[NETWORK_MAX_PEERS - 2];

                                uwb_wait_for_PASSIVES(entries, entry_pwr_diff_q8,
                                                      entry_antenna_unreliable, entry_ids,
                                                      &count, passive_msgs,
                                                      network_get_master());

                                for (uint8_t idx = 0; idx < count; idx++) {
                                    network_store_passive(idx, &passive_msgs[idx],
                                        &(uwb_rx_meas_t){ .ts            = entries[idx],
                                                          .pwr_diff_q8   = entry_pwr_diff_q8[idx] },
                                        entry_ids[idx], entry_antenna_unreliable[idx]);
                                }

                                if (!uwb_wait_for_FINAL()) {
                                    return UWB_TWR_TIMEOUT;
                                }
                                mprintf("[TWR] FINAL received\r\n");
                                return UWB_TWR_RECEIVED;

                            } else if (network_is_acknowledged()) {
                                /* ---- Passive observer path ---- */
                                mprintf("[TWR] P obs init=0x%04X resp=0x%04X\r\n",
                                        network_get_master(), rx_msg.receiver);

                                bool antenna_unreliable = false;
                                msg_passive_t passive_msg = {
                                    .seq_num          = rx_msg.data.poll.seq_num,
                                    .poll_rx_ts       = calibrate_rx_timestamp(rx_frame.rx_timestamp,
                                                                               rx_frame.rssi_q8,
                                                                               rx_msg.sender,
                                                                               &antenna_unreliable),
                                    .poll_pwr_diff_q8        = rx_frame.rssi_q8 - rx_frame.fp_q8,
                                    .poll_antenna_unreliable = antenna_unreliable,
                                    .IMU_vel_horiz           = speed_horiz,
                                    .IMU_vel_vert            = vel_z
                                };

                                if (!uwb_wait_for_RESPONSE(rx_msg.receiver,
                                                           &passive_msg.resp_rx_ts,
                                                           &passive_msg.resp_pwr_diff_q8,
                                                           &passive_msg.resp_antenna_unreliable))
                                    return UWB_TWR_TIMEOUT;

                                return uwb_send_PASSIVE(network_get_master(),
                                                        rx_msg.receiver,
                                                        &passive_msg);
                            }
                        } else {
                            if (uwb_unexpected_message_handler(&rx_msg)) {
                                return UWB_TWR_UNEXPECTED_MASTER;
                            }
                        }
                        break;
                    }
                    case DWM_RX_ERR:
                        mprintf("ERROR: TWR RX error 0x%08lX\r\n", rx_frame.status);
                        break;
                    case DWM_RX_TIMEOUT:
                        mprintf("ERROR: TWR window closed\r\n");
                        return UWB_TWR_TIMEOUT;
                }
            }
        }

        /* ---- Invalid or transitional sync states — skip ranging ---- */
        case UWB_SYNC_SLAVE_REPLY_FAILED:
        case UWB_SYNC_NEW_SLAVE:
        case UWB_SYNC_NEW_MASTER:
        case UWB_SYNC_TX_FAILED:
        default:
            return UWB_TWR_NOT_ENOUGH_DEVICES;
    }
}

/* -----------------------------------------------------------------------
 * Public API — uwb_build_share / uwb_read_share / uwb_share
 * ----------------------------------------------------------------------- */

/**
 * @brief Populate a @c msg_share_t from the current network state.
 *
 * Builds the canonical node list (self first, then peers in array order),
 * reads each node's IMU velocity, and fills all pair distances and accuracy
 * values in upper-triangle order.
 *
 * @param[out] out         Share struct to populate.
 * @param[in]  seq         Sequence number to embed.
 * @param[in]  sleep_time  Planned sleep duration in milliseconds.
 */
void uwb_build_share(msg_share_t *out, uint8_t seq, uint32_t sleep_time)
{
    memset(out, 0, sizeof(*out));
    out->seq_num    = seq;
    out->sleep_time = sleep_time;

    /* Build canonical node list: self first, then peers. */
    uint8_t n = 0;
    network_t *net = network_get_network();
    out->node_ids[n++] = net->self.id;
    for (uint8_t i = 0; i < net->count && n < NETWORK_MAX_PEERS; i++) {
        out->node_ids[n++] = net->peers[i].id;
    }
    out->node_count = n;

    /* Per-node IMU velocities. */
    for (uint8_t i = 0; i < n; i++) {
        uint16_t id = out->node_ids[i];
        if (id == net->self.id) {
            out->vel_vert[i]  = net->self.imu_vel_vert;
            out->vel_horiz[i] = net->self.imu_vel_horiz;
        } else {
            node_t *peer = find_peer(id);
            if (peer) {
                out->vel_vert[i]  = peer->imu_vel_vert;
                out->vel_horiz[i] = peer->imu_vel_horiz;
            }
        }
    }

    /* Per-pair distances and accuracy, upper-triangle order. */
    for (uint8_t i = 0; i < n; i++) {
        for (uint8_t j = i + 1u; j < n; j++) {
            uint8_t idx = (uint8_t)(i * (2u * n - i - 1u) / 2u + (j - i - 1u));
            out->distance_mm[idx] = network_get_distance(out->node_ids[i], out->node_ids[j]);
            out->accuracy[idx]    = network_get_certainty(out->node_ids[i], out->node_ids[j]);
        }
    }
}

/**
 * @brief Apply a received @c msg_share_t into the local network state.
 *
 * Updates IMU velocities for all nodes (including self), stores all pair
 * distances and overwrites certainty counters. Sentinel distances
 * (@c 0xFFFF) are skipped. Logs each pair update and per-node velocity.
 *
 * @param[in] in  Decoded SHARE message to apply.
 */
void uwb_read_share(const msg_share_t *in)
{
    uint8_t n = in->node_count;
    if (n > NETWORK_MAX_PEERS) n = NETWORK_MAX_PEERS;

    network_t *net = network_get_network();

    /* Update all node IMU velocities, including self. */
    for (uint8_t i = 0; i < n; i++) {
        uint16_t id = in->node_ids[i];
        if (id == net->self.id) {
            net->self.imu_vel_vert  = in->vel_vert[i];
            net->self.imu_vel_horiz = in->vel_horiz[i];
        } else {
            node_t *peer = find_peer(id);
            if (!peer) continue;
            peer->imu_vel_vert  = in->vel_vert[i];
            peer->imu_vel_horiz = in->vel_horiz[i];
        }
    }

    /* Update all pair distances and certainty. */
    for (uint8_t i = 0; i < n; i++) {
        for (uint8_t j = i + 1u; j < n; j++) {
            uint8_t idx = (uint8_t)(i * (2u * n - i - 1u) / 2u + (j - i - 1u));

            if (in->distance_mm[idx] == 0xFFFFu) continue;

            network_set_distance(in->node_ids[i], in->node_ids[j], in->distance_mm[idx]);
            network_set_distance(in->node_ids[j], in->node_ids[i], in->distance_mm[idx]);

            uint16_t stored = network_get_distance(in->node_ids[i], in->node_ids[j]);
            uint8_t  cert   = network_get_certainty(in->node_ids[i], in->node_ids[j]);
            mprintf("[SHARE_RX] stored=%u\n", stored);
            mprintf("[SHARE_RX] pair 0x%04X-0x%04X dist_raw=%5u stored=%u cert=%3u%s\n",
                    in->node_ids[i], in->node_ids[j],
                    in->distance_mm[idx], stored, cert,
                    in->distance_mm[idx] == 0xFFFFu ? " (sentinel/skipped)" : "");

            node_peer_state_t *s = network_get_peer_state(in->node_ids[i], in->node_ids[j]);
            if (s) s->certainty = in->accuracy[idx];

            node_peer_state_t *s_rev = network_get_peer_state(in->node_ids[j], in->node_ids[i]);
            if (s_rev) s_rev->certainty = in->accuracy[idx];
        }
    }

    mprintf("[SHARE_RX] seq=%u nodes=%u\n", in->seq_num, n);

    for (uint8_t i = 0; i < n; i++) {
        uint16_t id = in->node_ids[i];
        uint8_t vv, vh;
        if (id == net->self.id) {
            vv = net->self.imu_vel_vert;
            vh = net->self.imu_vel_horiz;
        } else {
            node_t *peer = find_peer(id);
            vv = peer ? peer->imu_vel_vert  : 0xFF;
            vh = peer ? peer->imu_vel_horiz : 0xFF;
        }
        mprintf("[SHARE_RX] node 0x%04X vel_vert=%3u vel_horiz=%3u\n", id, vv, vh);
    }
}

/**
 * @brief Broadcast or receive the SHARE message to close a ranging round.
 *
 * The responder/passive-timeout node builds and broadcasts a SHARE frame
 * carrying all distances, certainty values, and the next sleep duration.
 * All other nodes wait for the SHARE, apply it via @ref uwb_read_share,
 * and return an adjusted sleep time that accounts for early wake-up margin.
 *
 * @param etwr_result  Outcome of the preceding @ref uwb_extended_twr call.
 * @param sleep_time   Desired sleep duration in milliseconds (used if broadcasting).
 * @return Adjusted sleep time in milliseconds, 0 on timeout, or 0xFFFFFFFF on TX failure.
 */
uint32_t uwb_share(uwb_etwr_result_t etwr_result, uint32_t sleep_time)
{
    switch (etwr_result) {

        case UWB_TWR_RECEIVED:
        case UWB_TWR_PASSIVE_TIMEOUT:
        {
            osDelay(1);
            network_set_master(network_get_ownid());

            msg_share_t share;
            uwb_build_share(&share, network_get_expected_seq_num(), sleep_time);

            msg_t tx_msg = {
                .type        = MSG_TYPE_SHARE,
                .sender      = network_get_ownid(),
                .receiver    = ALL_ID,
                .data.share  = share,
            };

            dwm_tx_frame_t tx_frame = msg_encode_log(&tx_msg);

            if (dwm_tx(&tx_frame) != DWM_TX_OK) {
                if (dwm_tx(&tx_frame) != DWM_TX_OK) {
                    mprintf("ERROR: SHARE TX failed\r\n");
                    return 0xFFFFFFFF;
                }
            }

            mprintf("[SHARE] sent sleep=%lu ms nodes=%u\r\n",
                    sleep_time, share.node_count);
            return sleep_time;
        }

        case UWB_TWR_NOT_ENOUGH_DEVICES:
            return 0;

        case UWB_TWR_RECEIVED_PASSIVE:
        case UWB_TWR_EXCHANGE_COMPLETE:
        case UWB_TWR_NOTHING:
        case UWB_TWR_TX_FAILED:
        case UWB_TWR_TIMEOUT:
        case UWB_TWR_UNEXPECTED_MASTER:
        default:
        {
            dwm_rx_flush();
            dwm_rx_frame_t rx_frame = {0};
            uint32_t t_start = osKernelGetTickCount();
            bool first_rx = true;

            while (1) {
                uint32_t elapsed = osKernelGetTickCount() - t_start;
                if (elapsed >= T_SHARE_RX_WAIT) {
                    mprintf("ERROR: SHARE timeout\r\n");
                    return 0;
                }

                dwm_rx(&rx_frame, T_SHARE_RX_WAIT - elapsed, true, first_rx, NULL);
                first_rx = (rx_frame.type != DWM_RX_OK);

                switch (rx_frame.type) {
                    case DWM_RX_OK: {
                        msg_t rx_msg;
                        msg_decode_log(&rx_frame, &rx_msg);

                        if (rx_msg.type == MSG_TYPE_SHARE) {
                            if (!seq_ok(rx_msg.data.share.seq_num, "SHARE"))
                                break;
                            network_set_master(rx_msg.sender);
                            uwb_read_share(&rx_msg.data.share);

                            uint32_t adjusted = rx_msg.data.share.sleep_time - T_EARLY_WKUP;
                            mprintf("[SHARE] received sleep=%lu ms nodes=%u\r\n",
                                    adjusted, rx_msg.data.share.node_count);
                            return adjusted;
                        }
                        mprintf("ERROR: unexpected msg 0x%02X during SHARE wait\r\n", rx_msg.type);
                        if (uwb_unexpected_message_handler(&rx_msg)) {
                            return UWB_TWR_UNEXPECTED_MASTER;
                        }
                        break;
                    }
                    case DWM_RX_ERR:
                        mprintf("ERROR: SHARE RX error 0x%08lX\r\n", rx_frame.status);
                        break;
                    case DWM_RX_TIMEOUT:
                        mprintf("ERROR: SHARE timeout\r\n");
                        return 0;
                }
            }
        }
    }
}

/* -----------------------------------------------------------------------
 * Static helpers
 * ----------------------------------------------------------------------- */

/**
 * @brief Send a SYNC join reply to the master.
 *
 * Called by a slave whose ID was not yet in the master's peer list.
 * Applies @ref uwb_id_delay before transmitting to spread replies from
 * multiple slaves across time. On TX failure the frame is retransmitted once.
 *
 * @param sync_rx  The received SYNC message; the reply is directed at its sender.
 * @return true on successful transmission, false on double failure.
 */
static bool uwb_send_sync_reply(const msg_t *sync_rx)
{
    uwb_id_delay(network_get_ownid());

    msg_t tx_msg = {
        .type     = MSG_TYPE_SYNC,
        .sender   = network_get_ownid(),
        .receiver = sync_rx->sender,
        .data.sync = {
            .seq_num    = sync_rx->data.sync.seq_num,
            .peer_count = 0,
        },
    };

    dwm_tx_frame_t tx_frame = msg_encode_log(&tx_msg);

    if (dwm_tx(&tx_frame) != DWM_TX_OK) {
        if (dwm_tx(&tx_frame) != DWM_TX_OK) {
            mprintf("ERROR: SYNC reply TX failed\r\n");
            return false;
        }
    }

    mprintf("[SYNC] join reply - 0x%04X\r\n", sync_rx->sender);
    return true;
}

/**
 * @brief Build and broadcast a SYNC frame to all devices (@c ALL_ID).
 *
 * Fills the peer list via @ref network_fill_peer_ids, which excludes own ID
 * and inactive peers. On TX failure the frame is retransmitted once.
 *
 * @param seq_num  Sequence number to embed in the SYNC payload.
 * @return true on successful transmission, false on double failure.
 */
static bool uwb_sync_to_all_id(uint8_t seq_num)
{
    msg_sync_t sync_msg = {
        .seq_num    = seq_num,
        .peer_count = 0,
    };
    sync_msg.peer_count = network_fill_peer_ids(sync_msg.peer_ids, NETWORK_MAX_PEERS);

    msg_t tx_msg = {
        .type      = MSG_TYPE_SYNC,
        .sender    = network_get_ownid(),
        .receiver  = ALL_ID,
        .data.sync = sync_msg,
    };

    dwm_tx_frame_t tx_frame = msg_encode_log(&tx_msg);

    if (dwm_tx(&tx_frame) != DWM_TX_OK) {
        if (dwm_tx(&tx_frame) != DWM_TX_OK) {
            mprintf("ERROR: SYNC TX failed\r\n");
            return false;
        }
    }

    mprintf("[SYNC] sent seq=%d peers=%d\r\n", sync_msg.seq_num, sync_msg.peer_count);
    return true;
}

/**
 * @brief Transmit a POLL frame to the target responder.
 *
 * On TX failure the frame is retransmitted once.
 *
 * @param seq_num    Sequence number to embed.
 * @param target_id  Intended responder's node ID.
 * @param[out] out_poll_tx  Hardware TX timestamp of the POLL frame.
 * @return true on success, false on double TX failure.
 */
static bool uwb_send_POLL(uint8_t seq_num, uint16_t target_id, uint64_t *out_poll_tx)
{
    msg_t tx_msg = {
        .type      = MSG_TYPE_POLL,
        .sender    = network_get_ownid(),
        .receiver  = target_id,
        .data.poll = { .seq_num = seq_num },
    };

    dwm_tx_frame_t tx_frame = msg_encode_log(&tx_msg);

    if (dwm_tx(&tx_frame) != DWM_TX_OK) {
        if (dwm_tx(&tx_frame) != DWM_TX_OK) {
            mprintf("ERROR: POLL TX failed\r\n");
            return false;
        }
    }

    *out_poll_tx = tx_frame.tx_timestamp;
    return true;
}

/**
 * @brief Transmit a RESPONSE frame to the initiating master.
 *
 * On TX failure the frame is retransmitted once.
 *
 * @param seq_num    Sequence number to embed.
 * @param target_id  Master's node ID (RESPONSE receiver).
 * @param[out] out_resp_tx  Hardware TX timestamp of the RESPONSE frame.
 * @return true on success, false on double TX failure.
 */
static bool uwb_send_RESPONSE(uint8_t seq_num, uint16_t target_id, uint64_t *out_resp_tx)
{
    msg_t tx_msg = {
        .type          = MSG_TYPE_RESPONSE,
        .sender        = network_get_ownid(),
        .receiver      = target_id,
        .data.response = { .seq_num = seq_num },
    };

    dwm_tx_frame_t tx_frame = msg_encode_log(&tx_msg);

    if (dwm_tx(&tx_frame) != DWM_TX_OK) {
        if (dwm_tx(&tx_frame) != DWM_TX_OK) {
            mprintf("ERROR: RESPONSE TX failed\r\n");
            return false;
        }
    }

    *out_resp_tx = tx_frame.tx_timestamp;
    return true;
}

/**
 * @brief Transmit the FINAL frame using 9-bit aligned delayed TX.
 *
 * Pre-computes the exact hardware TX timestamp before encoding so that
 * the embedded @c final_tx_ts is accurate without a post-hoc correction.
 * The desired TX time is derived from the current system timer plus
 * @c T_FINAL_TX_ASAP_TICKS.
 *
 * Logs a non-zero delta between the desired and actual TX timestamps,
 * which indicates an OTP antenna delay misconfiguration.
 *
 * @param seq_num    Sequence number to embed.
 * @param target_id  Responder's node ID.
 * @param final_msg  FINAL payload; @c final_tx_ts is filled by this function.
 * @return true on success, false on late or failed scheduled TX.
 */
static bool uwb_send_FINAL(uint8_t seq_num, uint16_t target_id, msg_final_t *final_msg)
{
    uint32_t now_hi32        = dwt_readsystimestamphi32();
    uint64_t now             = (uint64_t)now_hi32 << 8;
    uint64_t final_tx_desired = now + T_FINAL_TX_ASAP_TICKS;

    /* Pre-compute the exact timestamp the hardware will produce.
     * dwt_setdelayedtrxtime drops the bottom 8 bits of the raw
     * (antenna-delay-subtracted) 40-bit value; the chip then adds antenna
     * delay back. Mirroring this here lets us embed the correct value into
     * the FINAL payload before encoding, avoiding up to ~30 cm ranging error. */
    uint16_t tx_ant_delay  = dwt_gettxantennadelay();
    uint64_t raw           = (final_tx_desired - tx_ant_delay) & 0xFFFFFFFFFFULL;
    uint64_t raw_aligned   = raw & 0xFFFFFFFE00ULL;
    uint64_t final_tx_exact = raw_aligned + tx_ant_delay;

    final_msg->final_tx_ts = calibrate_tx_timestamp(final_tx_exact);

    msg_t tx_msg = {
        .type        = MSG_TYPE_FINAL,
        .sender      = network_get_ownid(),
        .receiver    = target_id,
        .data.final  = *final_msg,
    };

    dwm_tx_frame_t tx_frame = msg_encode_log(&tx_msg);

    switch (dwm_tx_delayed(&tx_frame, final_tx_desired, T_FINAL_TX_WAIT_MS)) {
        case DWM_TX_LATE:
            mprintf("ERROR: FINAL TX scheduled too late\r\n");
            return false;
        case DWM_TX_ERROR:
            mprintf("ERROR: FINAL TX event timeout\r\n");
            return false;
        case DWM_TX_OK:
            break;
    }

    int32_t delta = (int32_t)(tx_frame.tx_timestamp - final_tx_exact);
    if (delta != 0)
        mprintf("ERROR: FINAL TX timestamp delta %ld ticks — check OTP antenna delay\r\n", delta);

    mprintf("[TWR] FINAL sent - 0x%04X seq=%d\r\n", target_id, seq_num);
    return true;
}

/**
 * @brief Wait for PASSIVE frames from all expected passive peers.
 *
 * Builds the expected sender list by excluding own ID and @p target_id from
 * the active peer list. Receives frames until all expected PASSIVEs arrive or
 * @c T_PASSIVE_TX_WAIT_MS × expected_count elapses. Silent peers are removed
 * from the network on timeout via @ref remove_silent_passive_peers.
 *
 * @param[out] entries              Calibrated RX timestamps, one per passive.
 * @param[out] entry_pwr_diff_q8    RSSI minus first-path power per passive (Q8).
 * @param[out] antenna_unreliable   Antenna quality flags per passive.
 * @param[out] entry_ids            Sender IDs per passive.
 * @param[out] out_count            Number of PASSIVEs successfully received.
 * @param[out] out_passive_msgs     Full decoded PASSIVE payloads.
 * @param[in]  target_id            Responder ID excluded from the expected list.
 * @return @c UWB_TWR_RECEIVED if all arrived, @c UWB_TWR_PASSIVE_TIMEOUT otherwise.
 */
static uwb_etwr_result_t uwb_wait_for_PASSIVES(uint64_t      entries[],
                                                int16_t       entry_pwr_diff_q8[],
                                                bool          antenna_unreliable[],
                                                uint16_t      entry_ids[],
                                                uint8_t      *out_count,
                                                msg_passive_t out_passive_msgs[],
                                                uint16_t      target_id)
{
    uint8_t        peer_count = 0;
    const node_t  *peers      = network_get_peers(&peer_count);

    uint16_t expected_ids[NETWORK_MAX_PEERS - 2];
    memset(expected_ids, 0, sizeof(expected_ids));
    uint8_t expected_count = 0;

    for (uint8_t i = 0; i < peer_count; i++) {
        if (peers[i].id == network_get_ownid()) continue;
        if (peers[i].id == target_id)           continue;
        expected_ids[expected_count++] = peers[i].id;
    }

    if (expected_count == 0) {
        *out_count = 0;
        return UWB_TWR_RECEIVED;
    }

    mprintf("[PAS] exp=%d\r\n", expected_count);
    for (uint8_t _pi = 0; _pi < expected_count; _pi++) {
        mprintf("[PAS] e%d=0x%04X\r\n", _pi, expected_ids[_pi]);
    }

    dwm_rx_flush();
    dwm_rx_frame_t rx_frame = {0};
    uint32_t t_start   = osKernelGetTickCount();
    uint8_t  idx       = 0;
    uint32_t wait_time = T_PASSIVE_TX_WAIT_MS * expected_count;
    bool     first_rx  = true;

    while (1) {
        uint32_t elapsed = osKernelGetTickCount() - t_start;
        if (elapsed >= wait_time) {
            remove_silent_passive_peers(expected_ids, expected_count, entry_ids, idx);
            *out_count = idx;
            if (idx < expected_count)
                mprintf("ERROR: PASSIVE timeout (%d/%d received)\r\n", idx, expected_count);
            return UWB_TWR_PASSIVE_TIMEOUT;
        }

        dwm_rx(&rx_frame, wait_time - elapsed, true, first_rx, NULL);
        first_rx = (rx_frame.type != DWM_RX_OK);

        switch (rx_frame.type) {
            case DWM_RX_OK: {
                msg_t rx_msg;
                msg_decode_log(&rx_frame, &rx_msg);

                if (rx_msg.type == MSG_TYPE_PASSIVE) {
                    if (!seq_ok(rx_msg.data.passive.seq_num, "PASSIVE"))
                        break;
                    if (rx_msg.receiver != network_get_master()) {
                        mprintf("ERROR: PASSIVE wrong receiver 0x%04X\r\n", rx_msg.receiver);
                        *out_count = idx;
                        return UWB_TWR_UNEXPECTED_MASTER;
                    }

                    bool single_antenna_unreliable = false;
                    entries[idx] = calibrate_rx_timestamp(rx_frame.rx_timestamp,
                                                           rx_frame.rssi_q8,
                                                           rx_msg.sender,
                                                           &single_antenna_unreliable);
                    entry_pwr_diff_q8[idx]   = rx_frame.rssi_q8 - rx_frame.fp_q8;
                    out_passive_msgs[idx]    = rx_msg.data.passive;
                    entry_ids[idx]           = rx_msg.sender;
                    antenna_unreliable[idx]  = single_antenna_unreliable;
                    mprintf("[PAS] rx%d=0x%04X\r\n", idx, rx_msg.sender);
                    idx++;

                    if (idx == expected_count) {
                        mprintf("[PASSIVE] all %d received\r\n", expected_count);
                        *out_count = idx;
                        return UWB_TWR_RECEIVED;
                    }
                } else {
                    if (uwb_unexpected_message_handler(&rx_msg)) {
                        return UWB_TWR_UNEXPECTED_MASTER;
                    }
                }
                break;
            }
            case DWM_RX_ERR:
                mprintf("ERROR: PASSIVE RX error 0x%08lX\r\n", rx_frame.status);
                break;
            case DWM_RX_TIMEOUT:
                remove_silent_passive_peers(expected_ids, expected_count, entry_ids, idx);
                *out_count = idx;
                if (idx < expected_count)
                    mprintf("ERROR: PASSIVE timeout (%d/%d received)\r\n", idx, expected_count);
                return UWB_TWR_PASSIVE_TIMEOUT;
        }
    }
}

/**
 * @brief Remove peers that were expected to transmit a PASSIVE but did not.
 *
 * Compares @p expected_ids against @p heard_ids and calls
 * @ref network_remove_peer for any ID not found in the heard list.
 *
 * @param expected_ids    IDs of nodes expected to transmit.
 * @param expected_count  Length of @p expected_ids.
 * @param heard_ids       IDs of nodes that were actually received.
 * @param heard_count     Length of @p heard_ids.
 */
static void remove_silent_passive_peers(const uint16_t *expected_ids,
                                        uint8_t         expected_count,
                                        const uint16_t *heard_ids,
                                        uint8_t         heard_count)
{
    for (uint8_t i = 0; i < expected_count; i++) {
        bool found = false;
        for (uint8_t j = 0; j < heard_count; j++) {
            if (heard_ids[j] == expected_ids[i]) { found = true; break; }
        }
        if (!found) {
            mprintf("ERROR: PASSIVE silent peer 0x%04X — removing\r\n", expected_ids[i]);
            network_remove_peer(expected_ids[i]);
        }
    }
}

/**
 * @brief Transmit this node's PASSIVE frame at the correct time-slot.
 *
 * Determines own slot index within the ordered passive-sender list
 * (peers excluding initiator and responder, in array order). Listens for
 * earlier slots' PASSIVE frames, filling @p passive_msg's entry arrays,
 * then schedules a 9-bit aligned delayed TX for this node's slot.
 *
 * A zero entry is stored for any earlier slot that times out or errors,
 * ensuring the entry count and IDs remain consistent.
 *
 * Logs a non-zero TX timestamp delta as an antenna delay misconfiguration.
 *
 * @param initiator_id  Master node ID (excluded from passive sender list).
 * @param responder_id  Responder node ID (excluded from passive sender list).
 * @param passive_msg   Pre-populated PASSIVE payload; entries are filled here.
 * @return @c UWB_TWR_RECEIVED_PASSIVE on success, @c UWB_TWR_TX_FAILED on error.
 */
static uwb_etwr_result_t uwb_send_PASSIVE(uint16_t      initiator_id,
                                           uint16_t      responder_id,
                                           msg_passive_t *passive_msg)
{
    uint8_t       peer_count = 0;
    const node_t *peers      = network_get_peers(&peer_count);

    uint16_t expected_ids[NETWORK_MAX_PEERS - 2] = {0};
    uint8_t  expected_count = 0;
    int8_t   my_idx         = -1;

    for (uint8_t i = 0; i < peer_count; i++) {
        if (peers[i].id == initiator_id) continue;
        if (peers[i].id == responder_id) continue;

        expected_ids[expected_count] = peers[i].id;

        if (peers[i].id == network_get_ownid()) {
            my_idx = expected_count;
            break;
        }
        expected_count++;
    }

    if (my_idx < 0) {
        mprintf("ERROR: PASSIVE cannot find own ID in peer list\r\n");
        return UWB_TWR_TX_FAILED;
    }

    mprintf("[PAS] send idx=%d pc=%d init=0x%04X resp=0x%04X\r\n",
            my_idx, peer_count, initiator_id, responder_id);

    dwm_rx_flush();
    dwm_rx_frame_t rx_frame = {0};
    uint32_t t_start    = osKernelGetTickCount();
    uint8_t  slot       = 0;
    uint8_t  idx        = 0;
    uint32_t wait_time  = T_PASSIVE_TX_WAIT_MS * my_idx;
    bool     first_rx   = true;

    while (slot < my_idx) {
        uint32_t elapsed = osKernelGetTickCount() - t_start;
        if (elapsed >= wait_time) {
            /* Ran out of time for pre-TX listening; stop and go transmit. */
            break;
        }

        dwm_rx(&rx_frame, wait_time - elapsed, true, first_rx, NULL);
        first_rx = (rx_frame.type != DWM_RX_OK);

        switch (rx_frame.type) {
            case DWM_RX_OK: {
                msg_t rx_msg;
                msg_decode_log(&rx_frame, &rx_msg);

                if (rx_msg.type     == MSG_TYPE_PASSIVE &&
                    rx_msg.receiver == network_get_master() &&
                    seq_ok(rx_msg.data.passive.seq_num, "PASSIVE")) {

                    bool single_antenna_unreliable = false;
                    passive_msg->entries[idx] = calibrate_rx_timestamp(
                        rx_frame.rx_timestamp,
                        rx_frame.rssi_q8,
                        rx_msg.sender,
                        &single_antenna_unreliable);
                    passive_msg->entry_pwr_diff_q8[idx]       = rx_frame.rssi_q8 - rx_frame.fp_q8;
                    passive_msg->entry_ids[idx]               = rx_msg.sender;
                    passive_msg->entry_antenna_unreliable[idx] = single_antenna_unreliable;
                } else {
                    if (uwb_unexpected_message_handler(&rx_msg)) {
                        return UWB_TWR_UNEXPECTED_MASTER;
                    }
                }
                idx++; slot++;
                break;
            }
            case DWM_RX_ERR: {
                mprintf("ERROR: PASSIVE RX error 0x%08lX\r\n", rx_frame.status);
                uint16_t lost_id = expected_ids[slot];
                passive_msg->entries[idx]                  = 0;
                passive_msg->entry_pwr_diff_q8[idx]        = 0;
                passive_msg->entry_ids[idx]                = lost_id;
                passive_msg->entry_antenna_unreliable[idx] = 0;
                idx++; slot++;
                break;
            }
            case DWM_RX_TIMEOUT: {
                mprintf("ERROR: PASSIVE pre-TX wait timeout (slot %d/%d)\r\n", slot, my_idx);
                uint16_t lost_id = expected_ids[slot];
                passive_msg->entries[idx]                  = 0;
                passive_msg->entry_pwr_diff_q8[idx]        = 0;
                passive_msg->entry_ids[idx]                = lost_id;
                passive_msg->entry_antenna_unreliable[idx] = 0;
                idx++; slot++;
                break;
            }
        }
    }

    passive_msg->entry_count = idx;

    /* Schedule TX — 9-bit aligned delayed TX to embed correct timestamp. */
    uint32_t now_hi32          = dwt_readsystimestamphi32();
    uint64_t now               = (uint64_t)now_hi32 << 8;
    uint64_t passive_tx_desired = now + T_PASSIVE_TX_ASAP_TICKS;

    uint16_t tx_ant_delay    = dwt_gettxantennadelay();
    uint64_t raw             = (passive_tx_desired - tx_ant_delay) & 0xFFFFFFFFFFULL;
    uint64_t raw_aligned     = raw & 0xFFFFFFFE00ULL;
    uint64_t passive_tx_exact = raw_aligned + tx_ant_delay;

    passive_msg->passive_tx_ts = calibrate_tx_timestamp(passive_tx_exact);

    msg_t tx_msg = {
        .type          = MSG_TYPE_PASSIVE,
        .sender        = network_get_ownid(),
        .receiver      = network_get_master(),
        .data.passive  = *passive_msg,
    };

    dwm_tx_frame_t tx_frame = msg_encode_log(&tx_msg);

    switch (dwm_tx_delayed(&tx_frame, passive_tx_desired, T_PASSIVE_TX_WAIT_MS)) {
        case DWM_TX_LATE:
            mprintf("ERROR: PASSIVE TX scheduled too late\r\n");
            return UWB_TWR_TX_FAILED;
        case DWM_TX_ERROR:
            mprintf("ERROR: PASSIVE TX event timeout\r\n");
            return UWB_TWR_TX_FAILED;
        case DWM_TX_OK:
            break;
    }

    int32_t delta = (int32_t)(tx_frame.tx_timestamp - passive_tx_exact);
    if (delta != 0)
        mprintf("ERROR: PASSIVE TX timestamp delta %ld ticks — check OTP antenna delay\r\n", delta);

    mprintf("[PASSIVE] sent idx=%d entries=%d\r\n", my_idx, passive_msg->entry_count);
    return UWB_TWR_RECEIVED_PASSIVE;
}