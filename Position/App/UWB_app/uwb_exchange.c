#include "cmsis_os.h"
#include "cmsis_os2.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "main.h"
#include "../Generic/my_print.h"
#include "DWM3000_setup.h"
#include "DWM3000_driver.h"
#include "messages.h"
#include "../UWB_app/uwb_network.h"
#include "uwb_exchange.h"
#include "position.h"

/* -----------------------------------------------------------------------
 * Forward declarations
 * ----------------------------------------------------------------------- */
static bool              uwb_send_sync_reply(const msg_t *sync_rx);
static bool              uwb_sync_to_all_id(uint8_t seq_num);
static bool              uwb_send_POLL(uint8_t seq_num, uint16_t target_id);
static bool              uwb_send_RESPONSE(uint8_t seq_num, uint16_t target_id);
static bool              uwb_send_FINAL(uint8_t seq_num, uint16_t target_id);
static uwb_etwr_result_t uwb_tdoa_receiver();
static uwb_etwr_result_t uwb_tdoa_sender();
static uwb_etwr_result_t uwb_tdoa_initiator();
static void remove_silent_passive_peers(const uint16_t *expected_ids, uint8_t expected_count, uint8_t heard_count);

/* -----------------------------------------------------------------------
 * Internal macros
 * ----------------------------------------------------------------------- */

/** @brief Construct a uwb_rx_meas_t inline from a dwm_rx_frame_t. */
#define RX_MEAS_FROM_FRAME(frame) \
    (uwb_rx_meas_t){ .ts = (frame).rx_timestamp, \
                     .rssi_q8 = (frame).rssi_q8, \
                     .fp_q8   = (frame).fp_q8 }

/** @brief Extract high 32 bits of a uint64_t for two-word mprintf. */
#define U64_HI(x)  ((uint32_t)((x) >> 32))

/** @brief Extract low 32 bits of a uint64_t for two-word mprintf. */
#define U64_LO(x)  ((uint32_t)((x) & 0xFFFFFFFFU))

/** @brief Print a 40-bit UWB timestamp label/value pair using two %08lX fields
 *  because mprintf does not support %llX. */
#define MPRINT_TS(label, val) \
    mprintf("UWB_eTWR_TEST - %-20s: 0x%08lX%08lX\r\n", \
            (label), U64_HI(val), U64_LO(val))

/* -----------------------------------------------------------------------
 * Internal helpers
 * ----------------------------------------------------------------------- */

 /**
 * @brief Sub-microsecond busy-wait using a hardware timer.
 *
 * @param us Delay in microseconds.
 */

static void delay_us(uint32_t us)
{
    __HAL_TIM_SET_COUNTER(&htim2, 0);
    while (__HAL_TIM_GET_COUNTER(&htim2) < us);
}

/**
 * @brief Staggered TX delay derived from device ID.
 *
 * Spreads devices across a 0–9999 µs window to reduce simultaneous
 * transmissions. Uses osDelay() for full milliseconds and delay_us()
 * for the sub-millisecond remainder.
 *
 * @param device_id This device's 16-bit network ID.
 */
static void uwb_id_delay(uint16_t device_id)
{
    uint32_t total_us = ((device_id & 0xFF) ^ (device_id >> 8)) % 10000;
    uint32_t ms = total_us / 1000;
    uint32_t us = total_us % 1000;

    if (ms > 0) osDelay(ms);
    if (us > 0) delay_us(us);
}

/**
 * @brief Check whether a given ID appears in a SYNC message's peer list.
 *
 * @param sync Pointer to the received sync payload.
 * @param id   Device ID to search for.
 * @return true if @p id is found, false otherwise.
 */
static bool sync_peer_contains(const msg_sync_t *sync, uint16_t id)
{
    for (int i = 0; i < sync->peer_count; i++)
        if (sync->peer_ids[i] == id) return true;
    return false;
}

#define SYNC_PAD(t_start)                                            \
    do {                                                             \
        uint32_t _e = osKernelGetTickCount() - (t_start);           \
        if (_e < T_SYNC_RX_ANSWER) osDelay(T_SYNC_RX_ANSWER - _e); \
    } while (0)

static bool seq_ok(uint8_t rx_seq, const char *tag)
{
    uint8_t exp = network_get_expected_seq_num();
    if (rx_seq != exp) {
        mprintf("%s - bad seq rx=%d exp=%d\r\n", tag, rx_seq, exp);
        return false;
    }
    return true;
}


/* -----------------------------------------------------------------------
 * Public API — uwb_sync
 * ----------------------------------------------------------------------- */

/**
 * @brief Perform one UWB synchronisation iteration.
 *
 * **Master behaviour:**
 * 1. Broadcasts a SYNC frame carrying the list of all known active peers.
 * 2. Listens for replies during a #T_SYNC_RX_ANSWER ms window.
 * 3. On the first valid reply, adds the sender as a new peer and returns
 *    #UWB_SYNC_MASTER.  Only one new peer is acknowledged per call — the
 *    caller is expected to call this function repeatedly until all peers
 *    are acknowledged.
 * 4. On dual-master detection the higher ID wins; the loser demotes itself
 *    and returns #UWB_SYNC_NEW_SLAVE.
 *
 * **Slave behaviour:**
 * 1. Listens indefinitely for a SYNC broadcast.
 * 2. If own ID is already in the peer list the device returns
 *    #UWB_SYNC_SLAVE_ACKNOWLEDGED.
 * 3. If own ID is not in the list the device sends a join reply and
 *    returns #UWB_SYNC_SLAVE_PENDING.
 * 4. On #SYNC_TIMEOUT_MAX consecutive timeouts the device promotes itself
 *    to master and returns #UWB_SYNC_NEW_MASTER.
 *
 * @note The DWM3000 must be awake before calling this function.
 *
 * @retval UWB_SYNC_MASTER            Master SYNC sent, one peer acknowledged.
 * @retval UWB_SYNC_MASTER_NO_REPLY   Master SYNC sent, no replies received.
 * @retval UWB_SYNC_SLAVE_ACKNOWLEDGED Own ID confirmed in master's peer list.
 * @retval UWB_SYNC_SLAVE_PENDING     Join reply sent, awaiting next cycle.
 * @retval UWB_SYNC_SLAVE_REPLY_FAILED Join reply TX failed twice.
 * @retval UWB_SYNC_NEW_MASTER        No master heard — promoted self.
 * @retval UWB_SYNC_NEW_SLAVE         Yielded to a higher-ID master.
 * @retval UWB_SYNC_TX_FAILED         SYNC TX failed at hardware level.
 */
uwb_sync_result_t uwb_sync()
{

    dwt_writesysstatuslo(DWT_INT_RXFTO_BIT_MASK    |   /* RX frame wait timeout */
                            DWT_INT_RXPTO_BIT_MASK);    /* RX SFD timeout */
    dwm_rx_flush();

    if (network_is_master()) {
        osDelay(2);

        uint8_t seq_num = network_get_expected_seq_num() + 1;
        network_set_expected_seq_num(seq_num);

        /* ---- MASTER: broadcast SYNC then collect one reply ---- */
        if (!uwb_sync_to_all_id(seq_num)) {
            mprintf("UWBsync [MASTER] - Failed to send SYNC, aborting\r\n");
            return UWB_SYNC_TX_FAILED;        /* TX never happened — no padding */
        }

        //start accelerometer calculation
        osThreadFlagsSet(AccelerometerHandle, 0x01);

        /* t_start anchored to SYNC transmission */
        uint32_t t_start = osKernelGetTickCount();
        dwm_rx_frame_t rx_frame = {0};

        while (1) {
            uint32_t elapsed = osKernelGetTickCount() - t_start;
            if (elapsed >= T_SYNC_RX_ANSWER) break;          /* window expired */

            dwm_rx(&rx_frame, T_SYNC_RX_ANSWER - elapsed, true);

            switch (rx_frame.type) {
            case DWM_RX_OK: {
                msg_t rx_msg;
                msg_decode(&rx_frame, &rx_msg);

                if (rx_msg.type != MSG_TYPE_SYNC) {
                    mprintf("UWBsync [MASTER] - Unexpected message type 0x%02X, ignoring\r\n",
                            rx_msg.type);
                    break;
                }

                if (rx_msg.receiver == network_get_ownid()) {
                    /* Valid join reply */
                    network_add_peer(rx_msg.sender);
                    mprintf("UWBsync [MASTER] - Peer 0x%04X acknowledged\r\n", rx_msg.sender);
                    SYNC_PAD(t_start);
                    return UWB_SYNC_MASTER;

                } else {
                    /* Dual-master conflict */
                    mprintf("UWBsync [MASTER] - Dual master detected sender 0x%04X\r\n",
                            rx_msg.sender);
                    if (rx_msg.sender > network_get_ownid()) {
                        network_set_master(rx_msg.sender);
                        mprintf("UWBsync [MASTER] - Yielding to 0x%04X\r\n", rx_msg.sender);
                        SYNC_PAD(t_start);
                        return UWB_SYNC_NEW_SLAVE;
                    }
                    mprintf("UWBsync [MASTER] - Other master 0x%04X should yield\r\n",
                            rx_msg.sender);
                }
                break;
            }

            case DWM_RX_ERR:
                mprintf("UWBsync [MASTER] - RX error 0x%08lX\r\n", rx_frame.status);
                break;

            case DWM_RX_TIMEOUT:
                break;    /* let while() decide — don't return early */
            }
        }

        /* Window expired naturally — already at T_SYNC_RX_ANSWER, no pad needed */
        mprintf("UWBsync [MASTER] - SYNC window closed\r\n");
        return UWB_SYNC_MASTER_NO_REPLY;

    } else {

        /* ---- SLAVE: wait for SYNC and attempt to join ---- */
        dwm_rx_frame_t rx_frame = {0};
        uint8_t timeout_count = 0;
        dwm_rx_flush();

        while (1) {
            osDelay(1);
            dwm_rx(&rx_frame, T_SYNC_RX_SET, true);

            switch (rx_frame.type) {
            case DWM_RX_OK: {
                msg_t rx_msg;
                msg_decode(&rx_frame, &rx_msg);

                switch (rx_msg.type) {

                case MSG_TYPE_SYNC:
                    if (rx_msg.receiver == ALL_ID) {

                        /* t_start anchored to SYNC receipt */
                        uint32_t t_start = osKernelGetTickCount();

                        //start accelerometer calculation
                        osThreadFlagsSet(AccelerometerHandle, 0x01);

                        /* Validate / update master */
                        if (rx_msg.sender != network_get_master()) {
                            if (rx_msg.sender > network_get_master()) {
                                network_set_master(rx_msg.sender);
                                mprintf("UWBsync [SLAVE] - New master 0x%04X\r\n", rx_msg.sender);
                                network_update_peers_from_sync(rx_msg.sender,
                                                               rx_msg.data.sync.peer_ids,
                                                               rx_msg.data.sync.peer_count);
                                network_set_expected_seq_num(rx_msg.data.sync.seq_num);
                            } else {
                                mprintf("UWBsync [SLAVE] - Unexpected master 0x%04X, expected 0x%04X\r\n",
                                        rx_msg.sender, network_get_master());
                                break;   /* discard — t_start not used, loop again */
                            }
                        } else {
                            network_update_peers_from_sync(rx_msg.sender,
                                                           rx_msg.data.sync.peer_ids,
                                                           rx_msg.data.sync.peer_count);
                            network_set_expected_seq_num(rx_msg.data.sync.seq_num);
                        }

                        if (sync_peer_contains(&rx_msg.data.sync, network_get_ownid())) {
                            mprintf("UWBsync [SLAVE] - Acknowledged by master 0x%04X\r\n",
                                    rx_msg.sender);
                            network_set_acknowledged(true);
                            SYNC_PAD(t_start);
                            return UWB_SYNC_SLAVE_ACKNOWLEDGED;

                        } else {
                            network_set_acknowledged(false);
                            mprintf("UWBsync [SLAVE] - Not acknowledged, sending reply\r\n");
                            if (!uwb_send_sync_reply(&rx_msg)) {
                                SYNC_PAD(t_start);
                                return UWB_SYNC_SLAVE_REPLY_FAILED;
                            }
                            SYNC_PAD(t_start);
                            return UWB_SYNC_SLAVE_PENDING;
                        }

                    } else {
                        mprintf("UWBsync [SLAVE] - Detected SYNC reply, waiting %d ms\r\n",
                                T_SYNC_TO_SYNC);
                        //osDelay(T_SYNC_TO_SYNC);
                    }
                    break;

                case MSG_TYPE_ERR:
                    mprintf("UWBsync [SLAVE] - Detected ERR message\r\n");
                    break;

                case MSG_TYPE_POLL:
                    mprintf("UWBsync [SLAVE] - Detected POLL, waiting %d ms\r\n", T_POLL_TO_SYNC);
                    //osDelay(T_POLL_TO_SYNC);
                    break;

                case MSG_TYPE_RESPONSE:
                    mprintf("UWBsync [SLAVE] - Detected RESPONSE, waiting %d ms\r\n",
                            T_RESPONSE_TO_SYNC);
                    //osDelay(T_RESPONSE_TO_SYNC);
                    break;

                case MSG_TYPE_FINAL:
                    mprintf("UWBsync [SLAVE] - Detected FINAL, waiting %d ms\r\n", T_FINAL_TO_SYNC);
                    //osDelay(T_FINAL_TO_SYNC);
                    break;

                default:
                    mprintf("UWBsync [SLAVE] - Unknown message type 0x%02X\r\n", rx_msg.type);
                    break;
                }
                break;
            }

            case DWM_RX_ERR:
                mprintf("UWBsync [SLAVE] - RX error 0x%08lX\r\n", rx_frame.status);
                break;

            case DWM_RX_TIMEOUT:
                timeout_count++;
                mprintf("UWBsync [SLAVE] - No SYNC received (timeout %d/%d)\r\n",
                        timeout_count, SYNC_TIMEOUT_MAX);
                if (timeout_count >= SYNC_TIMEOUT_MAX) {
                    uwb_id_delay(network_get_ownid());
                    network_set_master(network_get_ownid());
                    mprintf("UWBsync [SLAVE] - No master found, promoting to master\r\n");
                    return UWB_SYNC_NEW_MASTER;   /* no pad — no SYNC reference point */
                }
                break;
            }
        }
    }
}

/* -----------------------------------------------------------------------
 * Public API — uwb_extended_twr
 * ----------------------------------------------------------------------- */

/**
 * @brief Perform one DS-TWR ranging exchange or passive observation.
 *
 * The device's role within the exchange is determined by @p sync_result:
 *
 * - **Master** (#UWB_SYNC_MASTER / #UWB_SYNC_MASTER_NO_REPLY): selects the
 *   peer with the highest ranging uncertainty, sends POLL, waits for RESPONSE,
 *   then sends a delayed FINAL embedding all three initiator timestamps.
 *   On completion, hands the master role to the target.
 *
 * - **Slave acknowledged** (#UWB_SYNC_SLAVE_ACKNOWLEDGED): may be the POLL
 *   target (sends RESPONSE, waits for FINAL, extracts initiator timestamps
 *   from payload) or a passive observer (records all three frames for TDoA).
 *
 * - **Slave pending** (#UWB_SYNC_SLAVE_PENDING): treated identically to
 *   acknowledged for the TWR window — the device can still passively observe.
 *
 * - All other sync states: exchange is skipped.
 *
 * @note Timestamps are stored via the network measurement API and are
 *       available via network_get_twr() after the function returns.
 *
 * @param sync_result Role determined by the preceding uwb_sync() call.
 *
 * @retval UWB_TWR_EXCHANGE_COMPLETE  Master: exchange finished successfully.
 * @retval UWB_TWR_RECEIVED           Responder: all timestamps captured.
 * @retval UWB_TWR_RECEIVED_PASSIVE   Passive observer: full exchange captured.
 * @retval UWB_TWR_NOTHING            Exchange seen but device not involved.
 * @retval UWB_TWR_NOT_ENOUGH_DEVICES No peers or invalid sync state.
 * @retval UWB_TWR_TX_FAILED          TX of POLL, RESPONSE, or FINAL failed.
 * @retval UWB_TWR_TIMEOUT            RX window expired.
 * @retval UWB_TWR_UNEXPECTED_MASTER  Frame received from wrong sender.
 */


uwb_etwr_result_t uwb_extended_twr(uwb_sync_result_t sync_result)
{
    switch (sync_result) {

        /* ---- MASTER: initiates DS-TWR exchange ---- */
        case UWB_SYNC_MASTER:
        case UWB_SYNC_MASTER_NO_REPLY:
        {
            if (network_get_count() < 1) {
                mprintf("UWB_eTWR [MASTER] - No peers available\r\n");
                return UWB_TWR_NOT_ENOUGH_DEVICES;
            }

            network_reset_measurements();

            uint16_t target_id = network_get_highest_uncertainty();
            mprintf("UWB_eTWR [MASTER] - Ranging target: 0x%04X\r\n", target_id);

            dwm_rx_flush();

            osDelay(2); //wait for other to start listening

            if (!uwb_send_POLL(network_get_expected_seq_num(), target_id)) {
                mprintf("UWB_eTWR [MASTER] - Failed to send POLL\r\n");
                return UWB_TWR_TX_FAILED;
            }
            /* poll_tx timestamp saved inside uwb_send_POLL */

            dwm_rx_frame_t rx_frame = {0};
            uint32_t t_start = osKernelGetTickCount();

            while (1) {
                uint32_t elapsed = osKernelGetTickCount() - t_start;
                if (elapsed >= T_RESPONSE_RX_WAIT) {
                    mprintf("UWB_eTWR [MASTER] - RESPONSE window expired\r\n");
                    network_remove_peer(target_id);
                    mprintf("UWB_eTWR [MASTER] - Removing device 0x%04X from known peers\r\n", target_id);
                    return UWB_TWR_TIMEOUT;
                }

                dwm_rx(&rx_frame, T_RESPONSE_RX_WAIT - elapsed, true);

                switch (rx_frame.type) {
                case DWM_RX_OK: {
                    msg_t rx_msg;
                    msg_decode(&rx_frame, &rx_msg);

                    if (rx_msg.type == MSG_TYPE_RESPONSE &&
                        rx_msg.receiver == network_get_ownid() &&
                        rx_msg.sender   == target_id) {

                        if (!seq_ok(rx_msg.data.response.seq_num, "RESP")) {
                            break;
                        }

                        /* Save resp_rx — uwb_send_FINAL reads it via network_get_twr() */
                        network_set_twr_resp_rx(&RX_MEAS_FROM_FRAME(rx_frame));

                        if (!uwb_send_FINAL(network_get_expected_seq_num(), target_id)) {
                            mprintf("UWB_eTWR [MASTER] - Failed to send FINAL\r\n");
                            return UWB_TWR_TX_FAILED;
                        }
                        /* final_tx timestamp saved inside uwb_send_FINAL */

                        mprintf("UWB_eTWR [MASTER] - Exchange complete with 0x%04X\r\n", target_id);
                        /* Hand master role to target — it holds all timestamps
                         * and will compute the range. */
                        network_set_master(target_id);
                        return uwb_tdoa_initiator();


                    } else {
                        mprintf("UWB_eTWR [MASTER] - Unexpected msg 0x%02X from 0x%04X, ignoring\r\n",
                                rx_msg.type, rx_msg.sender);
                    }
                    break;
                }
                case DWM_RX_ERR:
                    mprintf("UWB_eTWR [MASTER] - RX error: 0x%08lX\r\n", rx_frame.status);
                    break;

                case DWM_RX_TIMEOUT:
                    mprintf("UWB_eTWR [MASTER] - RESPONSE window closed\r\n");
                    network_remove_peer(target_id);
                    return UWB_TWR_TIMEOUT;
                }
            }
        }

        /* ---- SLAVE: respond to or passively observe the DS-TWR exchange ---- */
        case UWB_SYNC_SLAVE_ACKNOWLEDGED:
        case UWB_SYNC_SLAVE_PENDING:
        {
            network_reset_measurements();
            dwm_rx_flush();

            dwm_rx_frame_t rx_frame = {0};
            uint32_t t_start = osKernelGetTickCount();

            while (1) {
                uint32_t elapsed = osKernelGetTickCount() - t_start;
                if (elapsed >= T_TWR_RX_WAIT) {
                    mprintf("UWB_eTWR [SLAVE] - TWR window expired\r\n");
                    return UWB_TWR_TIMEOUT;
                }

                dwm_rx(&rx_frame, T_TWR_RX_WAIT - elapsed, true);

                switch (rx_frame.type) {
                case DWM_RX_OK: {
                    msg_t rx_msg;
                    msg_decode(&rx_frame, &rx_msg);

                    switch (rx_msg.type) {

                    case MSG_TYPE_POLL:
                        if (rx_msg.sender != network_get_master()) {
                            mprintf("UWB_eTWR [SLAVE] - POLL from unexpected sender 0x%04X (master: 0x%04X)\r\n",
                                    rx_msg.sender, network_get_master());
                            return UWB_TWR_UNEXPECTED_MASTER;
                        }

                        if (rx_msg.receiver == network_get_ownid()) {
                            /* Responder path */

                            if (!seq_ok(rx_msg.data.poll.seq_num, "POLL")) {
                                break;
                            }

                            //mprintf("UWB_eTWR [SLAVE] - POLL received, sending RESPONSE\r\n");
                            network_set_twr_poll_rx(&RX_MEAS_FROM_FRAME(rx_frame));

                            if (!uwb_send_RESPONSE(network_get_expected_seq_num(), network_get_master())) {
                                mprintf("UWB_eTWR [SLAVE] - Failed to send RESPONSE\r\n");
                                return UWB_TWR_TX_FAILED;
                            }
                            /* resp_tx timestamp saved inside uwb_send_RESPONSE */

                        } else if (network_is_acknowledged()) {
                            /* Passive observer path */
                            if (!seq_ok(rx_msg.data.poll.seq_num, "POLL")) {
                                break;
                            }
                            mprintf("UWB_eTWR [SLAVE] - POLL observed passively (initiator: 0x%04X, target: 0x%04X)\r\n",
                                    rx_msg.sender, rx_msg.receiver);
                            network_set_obs_poll_rx(&RX_MEAS_FROM_FRAME(rx_frame));
                            //delay to cleanly add all the messages to the queue, no need to act on it
                            osDelay(2);
                        }
                        break;

                    case MSG_TYPE_RESPONSE:
                        /* RESPONSE should always be addressed back to the master */
                        if (rx_msg.receiver != network_get_master()) {
                            mprintf("UWB_eTWR [SLAVE] - RESPONSE to unexpected receiver 0x%04X\r\n",
                                    rx_msg.receiver);
                            return UWB_TWR_UNEXPECTED_MASTER;
                        }
                        if (network_is_acknowledged()) {

                            if (!seq_ok(rx_msg.data.response.seq_num, "RESP")) {
                                break;
                            }
                            mprintf("UWB_eTWR [SLAVE] - RESPONSE observed passively\r\n");
                            network_set_obs_resp_rx(&RX_MEAS_FROM_FRAME(rx_frame));
                            /* Do NOT return — FINAL still expected */
                        }
                        break;

                    case MSG_TYPE_FINAL:
                        if (rx_msg.sender != network_get_master()) {
                            mprintf("UWB_eTWR [SLAVE] - FINAL from unexpected sender 0x%04X\r\n",
                                    rx_msg.sender);
                            return UWB_TWR_UNEXPECTED_MASTER;
                        }

                        if (rx_msg.receiver == network_get_ownid()) {

                            if (!seq_ok(rx_msg.data.final.seq_num, "FINAL")) {
                                break;
                            }
                            /* Responder — exchange complete, compute range */
                            mprintf("UWB_eTWR [SLAVE] - FINAL received, exchange complete\r\n");

                            /* Save own RX measurement */
                            network_set_twr_final_rx(&RX_MEAS_FROM_FRAME(rx_frame));

                            /* Unpack initiator timestamps from FINAL payload —
                             * required by the DS-TWR ranging formula. */
                            network_set_twr_poll_tx(rx_msg.data.final.poll_tx_ts);
                            network_set_twr_final_tx(rx_msg.data.final.final_tx_ts);
                            network_set_twr_resp_rx(&(uwb_rx_meas_t){
                                .ts      = rx_msg.data.final.resp_rx_ts,
                                .rssi_q8 = rx_msg.data.final.resp_rssi_q8,
                                .fp_q8   = rx_msg.data.final.resp_fp_q8,
                            });

                            network_set_master(network_get_ownid());
                            return uwb_tdoa_receiver();

                        } else if (network_is_acknowledged()) {

                            if (!seq_ok(rx_msg.data.final.seq_num, "FINAL")) {
                                break;
                            }
                            /* Passive observer — full set of three frames captured */
                            mprintf("UWB_eTWR [SLAVE] - FINAL observed passively\r\n");
                            network_set_obs_final_rx(&RX_MEAS_FROM_FRAME(rx_frame));
                            network_set_master(rx_msg.receiver);
                            return uwb_tdoa_sender();

                        } else {
                            
                            return UWB_TWR_NOTHING;
                        }
                        /* break not reached */

                    default:
                        mprintf("UWB_eTWR [SLAVE] - Unexpected message type 0x%02X\r\n",
                                rx_msg.type);
                        break;
                    }
                    break;
                }

                case DWM_RX_ERR:
                    mprintf("UWB_eTWR [SLAVE] - RX error: 0x%08lX\r\n", rx_frame.status);
                    break;

                case DWM_RX_TIMEOUT:
                    mprintf("UWB_eTWR [SLAVE] - TWR window closed\r\n");
                    return UWB_TWR_TIMEOUT;
                }
            }
        }

        /* ---- Invalid or transitional sync states — skip ranging ---- */
        case UWB_SYNC_SLAVE_REPLY_FAILED:
        case UWB_SYNC_NEW_SLAVE:
        case UWB_SYNC_NEW_MASTER:   /* just promoted — no peers known yet */
        case UWB_SYNC_TX_FAILED:
            mprintf("UWB_eTWR - Invalid network state (%d), skipping ranging\r\n", sync_result);
            return UWB_TWR_NOT_ENOUGH_DEVICES;

        default:
            return UWB_TWR_NOT_ENOUGH_DEVICES;
    }
}

/**
 * @brief Broadcast or receive the post-exchange SHARE frame.
 *
 * The responder (#UWB_TWR_RECEIVED) broadcasts a SHARE frame carrying the
 * agreed sleep duration. All other roles wait and return the adjusted sleep
 * time (minus #T_EARLY_WKUP). Returns 0 on TX failure or RX timeout.
 *
 * @param etwr_result Result from the preceding uwb_extended_twr() call.
 * @param sleep_time  Sleep duration in ms to embed (responder only).
 * @return Adjusted sleep time on success, 0 on timeout or TX failure.
 */
uint32_t uwb_share (uwb_etwr_result_t etwr_result, uint32_t sleep_time){

    switch (etwr_result) {

        case UWB_TWR_RECEIVED:
        {
            osDelay(2);
            msg_share_t share_msg = {
                .seq_num    = network_get_expected_seq_num(),
                .sleep_time = sleep_time,
            };

            msg_t tx_msg = {
                .type      = MSG_TYPE_SHARE,
                .sender    = network_get_ownid(),
                .receiver  = ALL_ID,
                .data.share = share_msg,
            };

            dwm_tx_frame_t tx_frame = msg_encode(&tx_msg);

            if (dwm_tx(&tx_frame) != DWM_TX_OK) {
                mprintf("UWB_share [MASTER] - SHARE send failed, retrying\r\n");
                if (dwm_tx(&tx_frame) != DWM_TX_OK) {
                    mprintf("UWB_share [MASTER] - SHARE retry failed\r\n");
                    return 0;
                }
            }

            mprintf("UWB_share [MASTER] - SHARE sent\r\n");
            return sleep_time;
        }

        case UWB_TWR_RECEIVED_PASSIVE:
        case UWB_TWR_EXCHANGE_COMPLETE:
        case UWB_TWR_NOTHING:
        case UWB_TWR_NOT_ENOUGH_DEVICES:
        case UWB_TWR_TX_FAILED:
        case UWB_TWR_TIMEOUT:
        case UWB_TWR_UNEXPECTED_MASTER:
        default:

            dwm_rx_flush();
            dwm_rx_frame_t rx_frame = {0};
            uint32_t t_start = osKernelGetTickCount();

            while (1) {
                uint32_t elapsed = osKernelGetTickCount() - t_start;
                if (elapsed >= T_SHARE_RX_WAIT) {
                    mprintf("UWB_share [SLAVE] - SHARE window expired\r\n");
                    return 0;
                }

                dwm_rx(&rx_frame, T_SHARE_RX_WAIT - elapsed, true);

                switch (rx_frame.type) {
                case DWM_RX_OK: {
                    msg_t rx_msg;
                    msg_decode(&rx_frame, &rx_msg);

                    if (rx_msg.type == MSG_TYPE_SHARE) {
                        if (!seq_ok(rx_msg.data.share.seq_num, "SHARE")) {
                            break;
                        }

                        mprintf("UWB_share [SLAVE] - Set sleep time: 0x%04X\r\n", (rx_msg.data.share.sleep_time - T_EARLY_WKUP));
                        return (rx_msg.data.share.sleep_time - T_EARLY_WKUP);
                    }
                    else{
                        mprintf("UWB_share [SLAVE] - Unexpected message type 0x%02X, ignoring\r\n", rx_msg.type);
                    }
                    break;
                }
                case DWM_RX_ERR:
                    mprintf("UWB_share [SLAVE] - RX error: 0x%08lX\r\n", rx_frame.status);
                    break;

                case DWM_RX_TIMEOUT:
                    mprintf("UWB_share [SLAVE] - SHARE window closed\r\n");
                    return 0;
                

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
 * Called by a slave that received a SYNC broadcast but whose ID was not yet
 * in the master's peer list. Applies uwb_id_delay() before transmitting to
 * spread replies from multiple slaves across time. On TX failure the frame
 * is retransmitted once.
 *
 * @param sync_rx The received SYNC message; the reply is directed at its sender.
 * @return true on successful transmission, false on double failure.
 */
static bool uwb_send_sync_reply(const msg_t *sync_rx)
{
    uwb_id_delay(network_get_ownid());

    msg_sync_t reply_sync = {
        .seq_num    = sync_rx->data.sync.seq_num,
        .peer_count = 0,    /* Slave sends no peer info — master is the authority */
    };

    msg_t tx_msg = {
        .type      = MSG_TYPE_SYNC,
        .sender    = network_get_ownid(),
        .receiver  = sync_rx->sender,
        .data.sync = reply_sync,
    };

    dwm_tx_frame_t tx_frame = msg_encode(&tx_msg);

    if (dwm_tx(&tx_frame) != DWM_TX_OK) {
        mprintf("UWBsync [SLAVE] - SYNC reply send failed, retrying\r\n");
        if (dwm_tx(&tx_frame) != DWM_TX_OK) {
            mprintf("UWBsync [SLAVE] - SYNC reply retry failed\r\n");
            return false;
        }
    }

    mprintf("UWBsync [SLAVE] - SYNC reply sent to 0x%04X\r\n", sync_rx->sender);
    return true;
}

/**
 * @brief Build and broadcast a SYNC frame to all devices (ALL_ID).
 *
 * Fills the peer list from the current network state via
 * network_fill_peer_ids(), which excludes own ID and inactive peers.
 * On TX failure the frame is retransmitted once.
 *
 * @param seq_num Sequence number to embed in the SYNC payload.
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

    dwm_tx_frame_t tx_frame = msg_encode(&tx_msg);

    if (dwm_tx(&tx_frame) != DWM_TX_OK) {
        mprintf("UWBsync [MASTER] - SYNC send failed, retrying\r\n");
        if (dwm_tx(&tx_frame) != DWM_TX_OK) {
            mprintf("UWBsync [MASTER] - SYNC retry failed\r\n");
            return false;
        }
    }

    mprintf("UWBsync [MASTER] - SYNC sent (seq=%d, peers=%d)\r\n",
            sync_msg.seq_num, sync_msg.peer_count);
    return true;
}

/**
 * @brief Build and send a POLL frame to begin a DS-TWR exchange.
 *
 * Saves the TX timestamp into the network measurement layer via
 * network_set_twr_poll_tx() after successful transmission.
 *
 * @param seq_num   Sequence number for this exchange.
 * @param target_id Destination device ID (the ranging responder).
 * @return true on success, false if TX failed twice.
 */
static bool uwb_send_POLL(uint8_t seq_num, uint16_t target_id)
{
    msg_poll_t poll_msg = { .seq_num = seq_num };

    msg_t tx_msg = {
        .type      = MSG_TYPE_POLL,
        .sender    = network_get_ownid(),
        .receiver  = target_id,
        .data.poll = poll_msg,
    };

    dwm_tx_frame_t tx_frame = msg_encode(&tx_msg);

    if (dwm_tx(&tx_frame) != DWM_TX_OK) {
        mprintf("UWB_eTWR [MASTER] - POLL send failed, retrying\r\n");
        if (dwm_tx(&tx_frame) != DWM_TX_OK) {
            mprintf("UWB_eTWR [MASTER] - POLL retry failed\r\n");
            return false;
        }
    }

    network_set_twr_poll_tx(tx_frame.tx_timestamp);
    //cannot send it due to timing
    //mprintf("UWB_eTWR [MASTER] - POLL sent (seq=%d, target=0x%04X)\r\n", seq_num, target_id);
    return true;
}

/**
 * @brief Build and send a RESPONSE frame to the master.
 *
 * Saves the TX timestamp into the network measurement layer via
 * network_set_twr_resp_tx() after successful transmission.
 *
 * @param seq_num   Sequence number for this exchange.
 * @param target_id Destination device ID (the ranging initiator/master).
 * @return true on success, false if TX failed twice.
 */
static bool uwb_send_RESPONSE(uint8_t seq_num, uint16_t target_id)
{
    msg_response_t response_msg = { .seq_num = seq_num };

    msg_t tx_msg = {
        .type          = MSG_TYPE_RESPONSE,
        .sender        = network_get_ownid(),
        .receiver      = target_id,
        .data.response = response_msg,
    };

    dwm_tx_frame_t tx_frame = msg_encode(&tx_msg);

    if (dwm_tx(&tx_frame) != DWM_TX_OK) {
        mprintf("UWB_eTWR [SLAVE] - RESPONSE send failed, retrying\r\n");
        if (dwm_tx(&tx_frame) != DWM_TX_OK) {
            mprintf("UWB_eTWR [SLAVE] - RESPONSE retry failed\r\n");
            return false;
        }
    }

    network_set_twr_resp_tx(tx_frame.tx_timestamp);
    //cannot send due to timing
    //mprintf("UWB_eTWR [SLAVE] - RESPONSE sent (seq=%d, target=0x%04X)\r\n", seq_num, target_id);
    return true;
}


/**
 * @brief Build and send a delayed FINAL frame to complete the DS-TWR exchange.
 *
 * Pre-computes the exact hardware TX timestamp before encoding the message,
 * so that the embedded final_tx_ts in the payload exactly matches what the
 * radio will fire — eliminating the 8-bit truncation error that would
 * otherwise introduce up to ~30 cm of ranging error.
 *
 * The actual TX timestamp (from dwm_tx_delayed) is saved via
 * network_set_twr_final_tx() after transmission.
 *
 * @param seq_num   Sequence number for this exchange.
 * @param target_id Destination device ID (the ranging responder).
 * @return true on success, false if TX was late or timed out.
 */
static bool uwb_send_FINAL(uint8_t seq_num, uint16_t target_id)
{
    const twr_timestamps_t *twr = network_get_twr();

    uint64_t final_tx_desired = twr->resp_rx.ts + T_FINAL_TX_DELAY_TICKS;

    /* Pre-compute the exact timestamp the hardware will produce.
     * dwt_setdelayedtrxtime drops the bottom 8 bits of the raw
     * (antenna-delay-subtracted) 40-bit value; the chip then adds antenna
     * delay back. Mirroring this here lets us embed the correct value into
     * the FINAL payload before encoding, avoiding up to ~30 cm ranging error.
     *  */
    //TODO: confirm tx_ant_delay matches the value stored in DW3xxx OTP.
    uint16_t tx_ant_delay   = dwt_gettxantennadelay();
    uint64_t raw            = (final_tx_desired - tx_ant_delay) & 0xFFFFFFFFFFULL;
    uint64_t raw_aligned    = raw & 0xFFFFFFFE00ULL;    /* 9-bit boundary — hardware ignores bit 8 of 40-bit time */
    uint64_t final_tx_exact = raw_aligned + tx_ant_delay;

    msg_final_t final_msg = {
        .seq_num      = seq_num,
        .poll_tx_ts   = twr->poll_tx,
        .resp_rx_ts   = twr->resp_rx.ts,
        .final_tx_ts  = final_tx_exact,     /* exact predicted value, not desired */
        .resp_rssi_q8 = twr->resp_rx.rssi_q8,
        .resp_fp_q8   = twr->resp_rx.fp_q8,
    };

    msg_t tx_msg = {
        .type       = MSG_TYPE_FINAL,
        .sender     = network_get_ownid(),
        .receiver   = target_id,
        .data.final = final_msg,
    };

    dwm_tx_frame_t tx_frame = msg_encode(&tx_msg);

    switch (dwm_tx_delayed(&tx_frame, final_tx_desired, T_FINAL_TX_WAIT_MS)) {
        case DWM_TX_LATE:
            mprintf("UWB_eTWR [MASTER] - FINAL TX scheduled too late\r\n");
            return false;
        case DWM_TX_ERROR:
            mprintf("UWB_eTWR [MASTER] - FINAL TX event timeout\r\n");
            return false;
        case DWM_TX_OK:
            break;
    }

    /* Sanity check: delta should be 0 after pre-computation fix.
     * A non-zero delta indicates an antenna delay calibration mismatch.
     *  */
    //TODO: investigate if delta != 0 persists — may need OTP antenna delay read.
    int32_t delta = (int32_t)(tx_frame.tx_timestamp - final_tx_exact);
    if (delta != 0)
        mprintf("UWB_eTWR [MASTER] - FINAL TX unexpected delta: %ld ticks\r\n", delta);

    network_set_twr_final_tx(tx_frame.tx_timestamp);
    mprintf("UWB_eTWR [MASTER] - FINAL sent (seq=%d, target=0x%04X)\r\n", seq_num, target_id);
    return true;
}

/**
 * @brief Compute and publish a DS-TWR range estimate (responder side).
 *
 * Called after the responder has captured all six timestamps (three local,
 * three from the FINAL payload). Applies the standard DS-TWR formula:
 *
 *   Tprop = (Round1 × Round2 − Reply1 × Reply2) / (Round1 + Round2 + Reply1 + Reply2)
 *
 * where Round1 = resp_rx − poll_tx, Reply1 = resp_tx − poll_rx,
 *       Round2 = final_rx − resp_tx, Reply2 = final_tx − resp_rx.
 *
 * @param seq_num Sequence number of the completed exchange.
 * @return UWB_TWR_RECEIVED always (placeholder until publishing is implemented).
 *
 * @note 
 * @note 
 */

static uwb_etwr_result_t uwb_tdoa_receiver()
{   
    if (network_get_count() <= 2) {
        mprintf("PASSIVE_RX - too few nodes\r\n");
        return UWB_TWR_RECEIVED;
    }

    uint8_t expected_count = network_get_count() - 2;
    mprintf("PASSIVE_RX - expecting %d\r\n", expected_count);

    // create expected members id list
    uint8_t peer_count = 0;
    const node_t *peers = network_get_peers(&peer_count);

    uint16_t expected_ids[NETWORK_MAX_PEERS - 2];
    memset(expected_ids, 0, sizeof(expected_ids));
    uint8_t position = 0;

    for (uint8_t i = 1; i < peer_count; i++) {
        if (peers[i].id == network_get_ownid()) continue;
        expected_ids[position++] = peers[i].id;
    }

    dwm_rx_flush();

    dwm_rx_frame_t rx_frame = {0};
    uint32_t t_start   = osKernelGetTickCount();
    uint8_t  idx       = 0;
    uint32_t wait_time = T_PASSIVE_TX_WAIT_MS * expected_count;

    while (1) {
        uint32_t elapsed = osKernelGetTickCount() - t_start;
        if (elapsed >= wait_time) {
            mprintf("PASSIVE_RX - window expired (%d/%d)\r\n", idx, expected_count);
            remove_silent_passive_peers(expected_ids, position, idx);
            return UWB_TWR_TIMEOUT;
        }

        dwm_rx(&rx_frame, wait_time - elapsed, true);

        switch (rx_frame.type) {
        case DWM_RX_OK: {
            msg_t rx_msg;
            msg_decode(&rx_frame, &rx_msg);

            if (rx_msg.type == MSG_TYPE_PASSIVE) {
                if (!seq_ok(rx_msg.data.passive.seq_num, "PASSIVE")) {
                    break;
                }
                if (rx_msg.receiver != network_get_ownid()) {
                    mprintf("PASSIVE_RX - wrong rx 0x%04X\r\n", rx_msg.receiver);
                    return UWB_TWR_UNEXPECTED_MASTER;
                }
                mprintf("PASSIVE_RX - got 0x%04X\r\n", rx_msg.sender);

                network_set_passive_device_id(idx, rx_msg.sender);
                network_set_passive_ss_tx(idx, rx_msg.data.passive.passive_tx_ts);
                network_set_passive_ss_rx(idx, &RX_MEAS_FROM_FRAME(rx_frame));

                twr_observation_simple_t obs = {
                    .poll_rx  = rx_msg.data.passive.poll_rx_ts,
                    .resp_rx  = rx_msg.data.passive.resp_rx_ts,
                    .final_rx = rx_msg.data.passive.final_rx_ts,
                };
                network_set_passive_twr_observation(idx, &obs);

                passive_observation_t passive_obs = {0};
                for (uint8_t i = 0; i < rx_msg.data.passive.entry_count; i++)
                    passive_obs.passive_rx[i] = rx_msg.data.passive.entries[i];

                network_set_passive_observation(idx, &passive_obs);
                network_increment_passive_count();
                idx++;

                if (idx == expected_count) {
                    mprintf("PASSIVE_RX - all received\r\n");
                    return UWB_TWR_RECEIVED;
                }

                osDelay(2);
            } else {
                mprintf("PASSIVE_RX - skip type 0x%02X\r\n", rx_msg.type);
            }
            break;
        }
        case DWM_RX_ERR:
            mprintf("PASSIVE_RX - err 0x%08lX\r\n", rx_frame.status);
            break;

        case DWM_RX_TIMEOUT:
            mprintf("PASSIVE_RX - timeout\r\n");
            remove_silent_passive_peers(expected_ids, position, idx);
            return UWB_TWR_TIMEOUT;

        }
    }
    //return UWB_TWR_RECEIVED;
}
//TODO test this - need 4 nodes

/**
 * @brief Remove peers that failed to send a PASSIVE report this round.
 *
 * @param expected_ids  IDs of nodes expected to send PASSIVE.
 * @param expected_count Number of entries in expected_ids[].
 * @param heard_count   Number of PASSIVE reports actually received (idx).
 */
static void remove_silent_passive_peers(const uint16_t *expected_ids,
                                        uint8_t expected_count,
                                        uint8_t heard_count)
{
    for (uint8_t i = 0; i < expected_count; i++) {
        bool found = false;
        for (uint8_t j = 0; j < heard_count; j++) {
            if (network_get_passive_device_id(j) == expected_ids[i]) {
                found = true;
                break;
            }
        }
        if (!found) {
            mprintf("PASSIVE_RX - removing silent 0x%04X\r\n", expected_ids[i]);
            network_remove_peer(expected_ids[i]);
        }
    }
}

/**
 * @brief Listen for preceding PASSIVE reports, then transmit own (passive node side).
 *
 * Waits for all nodes ahead in the peer list to transmit their PASSIVE frame,
 * recording their RX timestamps, then transmits own PASSIVE to the master.
 *
 * @return UWB_TWR_RECEIVED_PASSIVE on success, UWB_TWR_TIMEOUT/TX_FAILED on error.
 */
static uwb_etwr_result_t uwb_tdoa_sender()
{
    //removing master from list, so devies are not waiting for him to send message - he never will
    //last entry on the list is now at his position, should not matter, as all devices do the same thing

    network_remove_peer(network_get_master());

    int8_t my_idx = network_get_peer_index(network_get_ownid());
    if (my_idx < 0) {
        mprintf("PASSIVE_TX - not in peer list\r\n");
        return UWB_TWR_TIMEOUT;
    }
    mprintf("PASSIVE_TX - idx %d, waiting %d\r\n", my_idx, my_idx - 1);

    dwm_rx_flush();

    dwm_rx_frame_t rx_frame = {0};
    uint32_t t_start   = osKernelGetTickCount();
    uint8_t  idx       = 0;
    uint32_t wait_time = T_PASSIVE_TX_WAIT_MS * (my_idx - 1);

    while (1) {
        if (idx == (my_idx - 1)) break;

        uint32_t elapsed = osKernelGetTickCount() - t_start;
        if (elapsed >= wait_time) {
            mprintf("PASSIVE_TX - rx window expired\r\n");
            return UWB_TWR_TIMEOUT;
        }

        dwm_rx(&rx_frame, wait_time - elapsed, true);

        switch (rx_frame.type) {
        case DWM_RX_OK: {
            msg_t rx_msg;
            msg_decode(&rx_frame, &rx_msg);

            if (rx_msg.type == MSG_TYPE_PASSIVE) {
                if (rx_msg.receiver != network_get_master()) {
                    mprintf("PASSIVE_TX - wrong master 0x%04X\r\n", rx_msg.receiver);
                    return UWB_TWR_UNEXPECTED_MASTER;
                }
                if (!seq_ok(rx_msg.data.passive.seq_num, "PASSIVE")) {
                    break;
                }
                uint64_t ts = position_calibrate_timestamp(rx_frame.rx_timestamp);
                network_set_passive_report_rx(idx, ts);
                idx++;

                osDelay(2); //listening to another message if not the message may come during antenna reinicialization

            }
            else {
                mprintf("PASSIVE_TX - skip type 0x%02X\r\n", rx_msg.type);
            }
            break;
        }

        case DWM_RX_ERR:
            mprintf("PASSIVE_TX - err 0x%08lX\r\n", rx_frame.status);
            break;

        case DWM_RX_TIMEOUT:
            mprintf("PASSIVE_TX - rx timeout\r\n");
            return UWB_TWR_TIMEOUT;

        }
    }

    /* ---- Build and send PASSIVE frame ---- */
    const twr_observation_t     *obs     = network_get_self_twr_observation();
    const passive_observation_t *passive = network_get_self_passive_observation();

    /* Schedule TX as soon as possible — just enough for SPI + DW3xxx startup. */
    uint32_t now_hi32           = dwt_readsystimestamphi32();
    uint64_t now                = (uint64_t)now_hi32 << 8;
    uint64_t passive_tx_desired = now + T_PASSIVE_TX_ASAP_TICKS;
    
    /* 9-bit alignment — mirrors DW3xxx hardware rounding, avoids ~30 cm error. */
    uint16_t tx_ant_delay     = dwt_gettxantennadelay();
    uint64_t raw              = (passive_tx_desired - tx_ant_delay) & 0xFFFFFFFFFFULL;
    uint64_t raw_aligned      = raw & 0xFFFFFFFE00ULL;
    uint64_t passive_tx_exact = raw_aligned + tx_ant_delay;

    /* Build entries[] from self_passive_observation — TX timestamps of
     * preceding PASSIVE frames heard before this node transmits. */
    uint8_t entry_count = network_get_self_passive_count();

    msg_passive_t passive_msg = {
        .seq_num       = network_get_expected_seq_num(),
        .poll_rx_ts    = obs->poll_rx.ts,
        .resp_rx_ts    = obs->resp_rx.ts,
        .final_rx_ts   = obs->final_rx.ts,
        .passive_tx_ts = passive_tx_exact,
        .entry_count   = entry_count,
    };

    for (uint8_t i = 0; i < entry_count; i++)
        passive_msg.entries[i] = passive->passive_rx[i];

    msg_t tx_msg = {
        .type         = MSG_TYPE_PASSIVE,
        .sender       = network_get_ownid(),
        .receiver     = network_get_master(),
        .data.passive = passive_msg,
    };

    dwm_tx_frame_t tx_frame = msg_encode(&tx_msg);

    switch (dwm_tx_delayed(&tx_frame, passive_tx_desired, T_PASSIVE_TX_WAIT_MS)) {
        case DWM_TX_LATE:
            mprintf("PASSIVE_TX - too late\r\n");
            return UWB_TWR_TX_FAILED;
        case DWM_TX_ERROR:
            mprintf("PASSIVE_TX - tx error\r\n");
            return UWB_TWR_TX_FAILED;
        case DWM_TX_OK:
            break;
    }

    int32_t delta = (int32_t)(tx_frame.tx_timestamp - passive_tx_exact);
    if (delta != 0)
        mprintf("PASSIVE_TX - delta %ld\r\n", delta);

    mprintf("PASSIVE_TX - sent seq=%d n=%d\r\n", network_get_expected_seq_num(), entry_count);
    return UWB_TWR_RECEIVED_PASSIVE;

    
}

/**
 * @brief Wait for all PASSIVE reports after initiating DS-TWR (initiator side).
 *
 * Does not store any data — just waits for the PASSIVE window to complete
 * so the medium is clear before the next SYNC cycle begins.
 *
 * @return UWB_TWR_EXCHANGE_COMPLETE on success, UWB_TWR_TIMEOUT on expiry.
 */
static uwb_etwr_result_t uwb_tdoa_initiator()
{
    if (network_get_count() <= 2) {
        mprintf("PASSIVE_WAIT - too few nodes\r\n");
        return UWB_TWR_EXCHANGE_COMPLETE;
    }

    uint8_t expected_count = network_get_count() - 2;
    mprintf("PASSIVE_WAIT - expecting %d\r\n", expected_count);

    dwm_rx_flush();

    dwm_rx_frame_t rx_frame = {0};
    uint32_t t_start   = osKernelGetTickCount();
    uint8_t  idx       = 0;
    uint32_t wait_time = T_PASSIVE_TX_WAIT_MS * expected_count;

    while (1) {
        uint32_t elapsed = osKernelGetTickCount() - t_start;
        if (elapsed >= wait_time) {
            mprintf("PASSIVE_WAIT - window expired (%d/%d)\r\n", idx, expected_count);
            return UWB_TWR_TIMEOUT;
        }

        dwm_rx(&rx_frame, wait_time - elapsed, true);

        switch (rx_frame.type) {
        case DWM_RX_OK: {
            msg_t rx_msg;
            msg_decode(&rx_frame, &rx_msg);

            if (rx_msg.type == MSG_TYPE_PASSIVE) {

                if (rx_msg.receiver != network_get_master()) {
                    mprintf("PASSIVE_WAIT - wrong rx 0x%04X\r\n", rx_msg.receiver);
                    return UWB_TWR_UNEXPECTED_MASTER;
                }
                if (!seq_ok(rx_msg.data.passive.seq_num, "PASS")) {
                    break;
                }
                idx++;
                if (idx == expected_count) {
                    mprintf("PASSIVE_WAIT - done\r\n");
                    return UWB_TWR_EXCHANGE_COMPLETE;
                }
                osDelay(2);
            } else {
                mprintf("PASSIVE_WAIT - skip type 0x%02X\r\n", rx_msg.type);
            }
            break;
        }
        case DWM_RX_ERR:
            mprintf("PASSIVE_WAIT - err 0x%08lX\r\n", rx_frame.status);
            break;
        case DWM_RX_TIMEOUT:
            mprintf("PASSIVE_WAIT - timeout\r\n");
            return UWB_TWR_TIMEOUT;
        }
    }
}


#ifdef UWB_DEBUG
/* -----------------------------------------------------------------------
 * Test utility
 * ----------------------------------------------------------------------- */

/**
 * @brief Test wrapper: call uwb_extended_twr() and print all timestamps.
 *
 * Prints the result code and the full twr_timestamps_t contents on every
 * completed exchange, using two-word 32-bit formatting because mprintf
 * does not support %llX.
 *
 * @param sync_result Forwarded to uwb_extended_twr().
 */
uwb_etwr_result_t uwb_twr_test(uwb_sync_result_t sync_result)
{
    mprintf("UWBeTWRTEST - seq=%d sync=%d\r\n", network_get_expected_seq_num() , sync_result);

    uwb_etwr_result_t result = uwb_extended_twr(sync_result);

    switch (result) {
        case UWB_TWR_EXCHANGE_COMPLETE:  mprintf("UWBeTWRTEST - EXCHANGE_COMPLETE\r\n");  break;
        case UWB_TWR_RECEIVED:           mprintf("UWBeTWRTEST - RECEIVED\r\n");           break;
        case UWB_TWR_RECEIVED_PASSIVE:   mprintf("UWBeTWRTEST - RECEIVED_PASSIVE\r\n");   break;
        case UWB_TWR_NOTHING:            mprintf("UWBeTWRTEST - NOTHING\r\n");            break;
        case UWB_TWR_TX_FAILED:          mprintf("UWBeTWRTEST - TX_FAILED\r\n");          break;
        case UWB_TWR_TIMEOUT:            mprintf("UWBeTWRTEST - TIMEOUT\r\n");            break;
        case UWB_TWR_UNEXPECTED_MASTER:  mprintf("UWBeTWRTEST - UNEXPECTED_MASTER\r\n");  break;
        case UWB_TWR_NOT_ENOUGH_DEVICES: mprintf("UWBeTWRTEST - NOT_ENOUGH_DEVICES\r\n"); break;
        default: mprintf("UWBeTWRTEST - UNKNOWN (%d)\r\n", result); break;
    }

    if (result == UWB_TWR_RECEIVED_PASSIVE) {
        const twr_observation_t *obs = network_get_self_twr_observation();
        mprintf("UWBeTWRTEST - ---- Role: PASSIVE ----\r\n");
        MPRINT_TS("obs.poll_rx",   obs->poll_rx.ts);
        MPRINT_TS("obs.resp_rx",   obs->resp_rx.ts);
        mprintf("UWBeTWRTEST - %-20s: %d (%.2f dBm)\r\n", "obs.resp_rx.rssi",
                obs->resp_rx.rssi_q8, obs->resp_rx.rssi_q8 / 256.0f);
        MPRINT_TS("obs.final_rx",  obs->final_rx.ts);
        mprintf("UWBeTWRTEST - %-20s: %d (%.2f dBm)\r\n", "obs.final_rx.rssi",
                obs->final_rx.rssi_q8, obs->final_rx.rssi_q8 / 256.0f);
        return result;
    }

    if (result != UWB_TWR_EXCHANGE_COMPLETE &&
        result != UWB_TWR_RECEIVED) return result;

    /* ---- Initiator / Responder dump ---- */
    const twr_timestamps_t *twr = network_get_twr();

    mprintf("UWBeTWRTEST - ---- Role: %s ----\r\n",
            result == UWB_TWR_EXCHANGE_COMPLETE ? "INITIATOR" : "RESPONDER");

    MPRINT_TS("poll_tx",       twr->poll_tx);
    MPRINT_TS("resp_rx.ts",    twr->resp_rx.ts);
    mprintf("UWBeTWRTEST - %-20s: %d (%.2f dBm)\r\n", "resp_rx.rssi_q8",
            twr->resp_rx.rssi_q8, twr->resp_rx.rssi_q8 / 256.0f);
    mprintf("UWBeTWRTEST - %-20s: %d (%.2f dBm)\r\n", "resp_rx.fp_q8",
            twr->resp_rx.fp_q8,   twr->resp_rx.fp_q8   / 256.0f);
    MPRINT_TS("final_tx",      twr->final_tx);
    MPRINT_TS("poll_rx.ts",    twr->poll_rx.ts);
    mprintf("UWBeTWRTEST - %-20s: %d (%.2f dBm)\r\n", "poll_rx.rssi_q8",
            twr->poll_rx.rssi_q8, twr->poll_rx.rssi_q8 / 256.0f);
    mprintf("UWBeTWRTEST - %-20s: %d (%.2f dBm)\r\n", "poll_rx.fp_q8",
            twr->poll_rx.fp_q8,   twr->poll_rx.fp_q8   / 256.0f);
    MPRINT_TS("resp_tx",       twr->resp_tx);
    MPRINT_TS("final_rx.ts",   twr->final_rx.ts);
    mprintf("UWBeTWRTEST - %-20s: %d (%.2f dBm)\r\n", "final_rx.rssi_q8",
            twr->final_rx.rssi_q8, twr->final_rx.rssi_q8 / 256.0f);
    mprintf("UWBeTWRTEST - %-20s: %d (%.2f dBm)\r\n", "final_rx.fp_q8",
            twr->final_rx.fp_q8,   twr->final_rx.fp_q8   / 256.0f);

        /* ---- Passive reports collected by responder ---- */
    if (result == UWB_TWR_RECEIVED) {
        uint8_t pcount = network_get_passive_count();
        mprintf("UWBeTWRTEST - ---- Passive reports: %d ----\r\n", pcount);
        for (uint8_t i = 0; i < pcount; i++) {
            mprintf("UWBeTWRTEST - [%d] id=0x%04X\r\n", i, network_get_passive_device_id(i));
            MPRINT_TS("  ss_tx",       network_get_passive_ss_tx(i));
            uwb_rx_meas_t ss_rx = network_get_passive_ss_rx(i);
            MPRINT_TS("  ss_rx",        ss_rx.ts);
            MPRINT_TS("  obs.poll_rx", network_get_passive_twr_obs_poll_rx(i));
            MPRINT_TS("  obs.resp_rx", network_get_passive_twr_obs_resp_rx(i));
            MPRINT_TS("  obs.final_rx",network_get_passive_twr_obs_final_rx(i));
        }
    }

    return result;
}
#endif
