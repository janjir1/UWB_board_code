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

//TODO create function in driver to force trf off. When reception was successful the antenna currently
// turns off only after timeout (or at the last time when dwm_rx is called, do not reinitiate)
//TODO reinitiating the dwm3000 sucks try to use some integrated hw function - do only when extra time is available 
//TODO after promoting to master, domote self when not recieving sync answer
//TODO count tx failed and if too many, reset MCU
//TODO accel is newer written to network_t on masters side before share - needs the float to 8_t coversion
//TODO implement sleep
//TODO when waiting for SHARE we can ho to idle_rc

/* -----------------------------------------------------------------------
 * Forward declarations
 * ----------------------------------------------------------------------- */
static bool uwb_send_sync_reply(const msg_t *sync_rx);
static bool uwb_sync_to_all_id(uint8_t seq_num);
static bool uwb_send_POLL(uint8_t seq_num, uint16_t target_id, uint64_t *out_poll_tx);
static bool uwb_send_RESPONSE(uint8_t seq_num, uint16_t target_id, uint64_t *out_resp_tx);
static bool uwb_send_FINAL(uint8_t seq_num, uint16_t target_id, msg_final_t *final_msg);

static void remove_silent_passive_peers(const uint16_t *expected_ids,
                                        uint8_t expected_count,
                                        const uint16_t *heard_ids,
                                        uint8_t heard_count);

static uwb_etwr_result_t uwb_wait_for_PASSIVES(uint64_t entries[],
                                               int16_t entry_rssi_q8[],
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

/** @brief Construct a uwb_rx_meas_t inline from a dwm_rx_frame_t. */
#define RX_MEAS_FROM_FRAME(frame) \
    (uwb_rx_meas_t){ .ts = (frame).rx_timestamp, \
                     .pwr_diff_q8 = (frame).rssi_q8 - (frame).fp_q8 }

/** @brief Extract high 32 bits of a uint64_t for two-word mprintf. */
#define U64_HI(x) ((uint32_t)((x) >> 32))

/** @brief Extract low 32 bits of a uint64_t for two-word mprintf. */
#define U64_LO(x) ((uint32_t)((x) & 0xFFFFFFFFU))

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
 * Spreads devices across a 0-9999 us window to reduce simultaneous
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

#define SYNC_PAD(t_start) \
    do { \
        uint32_t _e = osKernelGetTickCount() - (t_start); \
        if (_e < T_SYNC_RX_ANSWER) osDelay(T_SYNC_RX_ANSWER - _e); \
    } while (0)

static bool seq_ok(uint8_t rx_seq, const char *tag)
{
    uint8_t exp = network_get_expected_seq_num();
    if (rx_seq != exp) {
        mprintf("ERROR: bad seq [%s] rx=%d exp=%d\r\n", tag, rx_seq, exp);
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
 * Master broadcasts SYNC and listens for replies. Slaves listen for SYNC, 
 * and reply if they are not in the master's peer list.
 *
 * @return Outcome and role for the subsequent TWR phase.
 */
uwb_sync_result_t uwb_sync()
{
    dwt_writesysstatuslo(DWT_INT_RXFTO_BIT_MASK |
                         DWT_INT_RXPTO_BIT_MASK);
    dwm_rx_flush();

    if (network_is_master()) {

        uint8_t seq_num = network_get_expected_seq_num() + 1;
        network_set_expected_seq_num(seq_num);
        osDelay(2);

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

            dwm_rx(&rx_frame, T_SYNC_RX_ANSWER - elapsed, true, first_rx);
            first_rx = (rx_frame.type != DWM_RX_OK);

            switch (rx_frame.type) {
            case DWM_RX_OK: {
                msg_t rx_msg;
                msg_decode(&rx_frame, &rx_msg);

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
                    /* Dual-master conflict — higher ID wins */
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
        uint8_t timeout_count = 0;
        dwm_rx_flush();
        bool first_rx = true;
        while (1) {
            osDelay(1);
            dwm_rx(&rx_frame, T_SYNC_RX_SET, true, first_rx);
            first_rx = (rx_frame.type != DWM_RX_OK);

            switch (rx_frame.type) {
            case DWM_RX_OK: {
                msg_t rx_msg;
                msg_decode(&rx_frame, &rx_msg);

                switch (rx_msg.type) {

                case MSG_TYPE_SYNC:
                    if (rx_msg.receiver == ALL_ID) {

                        uint32_t t_start = osKernelGetTickCount();
                        osThreadFlagsSet(AccelerometerHandle, 0x01);

                        if (rx_msg.sender != network_get_master()) {
                            if (rx_msg.sender > network_get_master()) {
                                network_set_master(rx_msg.sender);
                                network_update_peers_from_sync(rx_msg.sender,
                                                               rx_msg.data.sync.peer_ids,
                                                               rx_msg.data.sync.peer_count);
                                network_set_expected_seq_num(rx_msg.data.sync.seq_num);
                            } else {
                                mprintf("ERROR: SYNC from unexpected master 0x%04X (exp 0x%04X)\r\n",
                                        rx_msg.sender, network_get_master());
                                break;
                            }
                        } else {
                            network_update_peers_from_sync(rx_msg.sender,
                                                           rx_msg.data.sync.peer_ids,
                                                           rx_msg.data.sync.peer_count);
                            network_set_expected_seq_num(rx_msg.data.sync.seq_num);
                        }

                        if (sync_peer_contains(&rx_msg.data.sync, network_get_ownid())) {
                            mprintf("[SYNC] ack by master 0x%04X\r\n", rx_msg.sender);
                            network_set_acknowledged(true);
                            mprintf("[SYNC] entering pad\r\n");
                            SYNC_PAD(t_start);
                            mprintf("[SYNC] pad done\r\n"); 
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
                if (timeout_count >= SYNC_TIMEOUT_MAX) {
                    uwb_id_delay(network_get_ownid());
                    network_set_master(network_get_ownid());
                    mprintf("[SYNC] no master found — promoted to master\r\n");
                    return UWB_SYNC_NEW_MASTER;
                }
                break;
            }
        }
    }
}

static bool uwb_wait_for_RESPONSE(uint16_t target_id,
                                   uint64_t *out_resp_rx_ts,
                                   int16_t *out_resp_pwr_diff_q8)
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

        dwm_rx(&rx_frame, T_RESPONSE_RX_WAIT - elapsed, true, first_rx);
        first_rx = (rx_frame.type != DWM_RX_OK);;

        switch (rx_frame.type) {
        case DWM_RX_OK: {
            msg_t rx_msg;
            msg_decode(&rx_frame, &rx_msg);

            if (rx_msg.type == MSG_TYPE_RESPONSE &&
                rx_msg.receiver == network_get_master() &&
                rx_msg.sender == target_id) {

                if (!seq_ok(rx_msg.data.response.seq_num, "RESP"))
                    break;

                *out_resp_rx_ts  = rx_frame.rx_timestamp;
                *out_resp_pwr_diff_q8 = rx_frame.rssi_q8 - rx_frame.fp_q8;
                return true;
            }
            mprintf("ERROR: unexpected msg 0x%02X from 0x%04X during RESPONSE wait\r\n",
                    rx_msg.type, rx_msg.sender);
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

static bool uwb_wait_for_FINAL()
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

        dwm_rx(&rx_frame, T_FINAL_RX_WAIT - elapsed, true, first_rx);
        first_rx = (rx_frame.type != DWM_RX_OK);;

        switch (rx_frame.type) {
        case DWM_RX_OK: {
            msg_t rx_msg;
            msg_decode(&rx_frame, &rx_msg);

            if (rx_msg.type == MSG_TYPE_FINAL &&
                rx_msg.receiver == network_get_ownid() &&
                rx_msg.sender == network_get_master()) {

                if (!seq_ok(rx_msg.data.final.seq_num, "FINAL"))
                    break;

                network_store_final(&rx_msg.data.final,
                                    &(uwb_rx_meas_t){ .ts = rx_frame.rx_timestamp,
                                                        .pwr_diff_q8 = rx_frame.rssi_q8 - rx_frame.fp_q8 });
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

uwb_etwr_result_t uwb_extended_twr(uwb_sync_result_t sync_result)
{
    network_reset_measurements();
    dwm_rx_flush();

    switch (sync_result) {

    /* ---- MASTER: initiates DS-TWR exchange ---- */
    case UWB_SYNC_MASTER:
    case UWB_SYNC_MASTER_NO_REPLY:
    {
        if (network_get_count() < 1) {
            mprintf("ERROR: no peers — skipping TWR\r\n");
            return UWB_TWR_NOT_ENOUGH_DEVICES;
        }
        network_print_certainty();    
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
            .poll_tx_ts = poll_tx,
        };

        if (!uwb_wait_for_RESPONSE(target_id, &final_msg.resp_rx_ts, &final_msg.resp_pwr_diff_q8))
            return UWB_TWR_TIMEOUT;

        msg_passive_t passive_msgs[NETWORK_MAX_PEERS - 2];
        uwb_wait_for_PASSIVES(final_msg.entries, final_msg.entry_pwr_diff_q8,
                              final_msg.entry_id, &final_msg.entry_count,
                              passive_msgs, target_id);

        if (!uwb_send_FINAL(network_get_expected_seq_num(), target_id, &final_msg))
            return UWB_TWR_TX_FAILED;

        //network_set_master(target_id);
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
        bool first_rx = true;
        while (1) {
            uint32_t elapsed = osKernelGetTickCount() - t_start;
            if (elapsed >= T_TWR_RX_WAIT) {
                mprintf("ERROR: TWR window expired\r\n");
                return UWB_TWR_TIMEOUT;
            }

            dwm_rx(&rx_frame, T_TWR_RX_WAIT - elapsed, true, first_rx);
            first_rx = (rx_frame.type != DWM_RX_OK);;

            switch (rx_frame.type) {
            case DWM_RX_OK: {
                msg_t rx_msg;
                msg_decode(&rx_frame, &rx_msg);

                if (rx_msg.type == MSG_TYPE_POLL) {

                    if (rx_msg.sender != network_get_master()) {
                        mprintf("ERROR: POLL from unexpected sender 0x%04X (master: 0x%04X)\r\n",
                                rx_msg.sender, network_get_master());
                        return UWB_TWR_UNEXPECTED_MASTER;
                    }

                    if (!seq_ok(rx_msg.data.poll.seq_num, "POLL"))
                        return UWB_TWR_TIMEOUT;

                    if (rx_msg.receiver == network_get_ownid()) {
                        /* ---- Responder path ---- */
                        network_store_poll(&rx_msg.data.poll, &RX_MEAS_FROM_FRAME(rx_frame));

                        delay_us(50);
                        //osDelay(2);

                        uint64_t resp_tx;
                        if (!uwb_send_RESPONSE(network_get_expected_seq_num(),
                                               network_get_master(), &resp_tx)) {
                            mprintf("ERROR: RESPONSE TX failed\r\n");
                            return UWB_TWR_TX_FAILED;
                        }

                        mprintf("[TWR] RESPONDER - 0x%04X\r\n", rx_msg.sender);
                        network_store_resp_tx(resp_tx);

                        uint64_t entries[NETWORK_MAX_PEERS - 2];
                        int16_t  entry_pwr_diff_q8[NETWORK_MAX_PEERS - 2];
                        uint16_t entry_ids[NETWORK_MAX_PEERS - 2];
                        uint8_t  count = 0;
                        msg_passive_t passive_msgs[NETWORK_MAX_PEERS - 2];

                        uwb_wait_for_PASSIVES(entries, entry_pwr_diff_q8, entry_ids,
                                              &count, passive_msgs, network_get_master());

                        for (uint8_t idx = 0; idx < count; idx++) {
                            network_store_passive(idx, &passive_msgs[idx],
                                                  &(uwb_rx_meas_t){ .ts = entries[idx],
                                                                    .pwr_diff_q8 = entry_pwr_diff_q8[idx] },
                                                  entry_ids[idx]);
                        }

                        if (!uwb_wait_for_FINAL()) {
                            return UWB_TWR_TIMEOUT;  // Abort! Do not run distance calculations.
                        }
                        mprintf("[TWR] FINAL received\r\n");
                        return UWB_TWR_RECEIVED;

                    } else if (network_is_acknowledged()) {
                        /* ---- Passive observer path ---- */
                        msg_passive_t passive_msg = {
                            .seq_num      = rx_msg.data.poll.seq_num,
                            .poll_rx_ts   = rx_frame.rx_timestamp,
                            .poll_pwr_diff_q8 = rx_frame.rssi_q8 - rx_frame.fp_q8,
                        };

                        if (!uwb_wait_for_RESPONSE(rx_msg.receiver,
                                                   &passive_msg.resp_rx_ts,
                                                   &passive_msg.resp_pwr_diff_q8))
                            return UWB_TWR_TIMEOUT;

                        uwb_etwr_result_t result = uwb_send_PASSIVE(network_get_master(),
                                                                     rx_msg.receiver,
                                                                     &passive_msg);
                        //network_set_master(rx_msg.receiver);
                        return result;
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

/**
 * @brief Populate a msg_share_t from the current network state.
 *
 * Builds the canonical node list (self first, then peers in array order),
 * reads each node's IMU velocity from their peer slot relative to self,
 * and fills all pair distances + accuracy in upper-triangle order.
 *
 * @param[out] out     Share struct to populate.
 * @param[in]  seq     Sequence number to embed.
 * @param[in]  sleep   Planned sleep time in ms.
 */
void uwb_build_share(msg_share_t *out, uint8_t seq, uint32_t sleep_time)
{
    memset(out, 0, sizeof(*out));
    out->seq_num    = seq;
    out->sleep_time = sleep_time;

    /* ── Build canonical node list: self first, then peers ── */
    uint8_t n = 0;
    network_t *net = network_get_network();
    out->node_ids[n++] = net->self.id;
    for (uint8_t i = 0; i < net->count && n < NETWORK_MAX_PEERS; i++) {
        out->node_ids[n++] = net->peers[i].id;
    }
    out->node_count = n;

    /* ---> DIAGNOSTIC MPRINTF FOR BUG 3 <--- */
    mprintf("[DIAG_SHARE] Building SHARE msg. Nodes (%u): ", out->node_count);
    for (uint8_t d = 0; d < out->node_count; d++) {
        mprintf("0x%04X ", out->node_ids[d]);
    }
    mprintf("\r\n");
    /* -------------------------------------- */

    /* ── Per-node IMU velocities ── */
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

    /* ── Per-pair distances + accuracy, upper-triangle order ── */
    for (uint8_t i = 0; i < n; i++) {
        for (uint8_t j = i + 1u; j < n; j++) {
            uint8_t idx = (uint8_t)(i * (2u * n - i - 1u) / 2u + (j - i - 1u));
            out->distance_mm[idx] = network_get_distance(out->node_ids[i], out->node_ids[j]);
            out->accuracy[idx] = network_get_certainty(out->node_ids[i], out->node_ids[j]);
        }
    }
}

void uwb_read_share(const msg_share_t *in)
{
    uint8_t n = in->node_count;
    if (n > NETWORK_MAX_PEERS) n = NETWORK_MAX_PEERS;

    network_t *net = network_get_network();

    /* Update ALL node IMU velocities including self */
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

    /* Update ALL pair distances and certainty */
    for (uint8_t i = 0; i < n; i++) {
        for (uint8_t j = i + 1u; j < n; j++) {
            uint8_t idx = (uint8_t)(i * (2u * n - i - 1u) / 2u + (j - i - 1u));

            if (in->distance_mm[idx] == 0xFFFFu) continue;

            network_set_distance(in->node_ids[i], in->node_ids[j], in->distance_mm[idx]);
            network_set_distance(in->node_ids[j], in->node_ids[i], in->distance_mm[idx]);

            node_peer_state_t *s = network_get_peer_state(in->node_ids[i], in->node_ids[j]);
            if (s) s->certainty = in->accuracy[idx];

            node_peer_state_t *s_rev = network_get_peer_state(in->node_ids[j], in->node_ids[i]);
            if (s_rev) s_rev->certainty = in->accuracy[idx];
        }
    }

    /* ── Readback verification ── */
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
        mprintf("[SHARE_RX]   node 0x%04X  vel_vert=%3u vel_horiz=%3u\n", id, vv, vh);
    }

    for (uint8_t i = 0; i < n; i++) {
        for (uint8_t j = i + 1u; j < n; j++) {
            uint8_t idx = (uint8_t)(i * (2u * n - i - 1u) / 2u + (j - i - 1u));
            double stored = network_get_distance(in->node_ids[i], in->node_ids[j]);
            uint8_t cert  = network_get_certainty(in->node_ids[i], in->node_ids[j]);
            mprintf("[SHARE_RX]   pair 0x%04X-0x%04X  dist_raw=%5u stored=%.1f cert=%3u%s\n",
                    in->node_ids[i], in->node_ids[j],
                    in->distance_mm[idx],
                    stored,
                    cert,
                    in->distance_mm[idx] == 0xFFFFu ? " (sentinel/skipped)" : "");
        }
    }
}

uint32_t uwb_share(uwb_etwr_result_t etwr_result, uint32_t sleep_time)
{
    switch (etwr_result) {

    case UWB_TWR_RECEIVED:
    {
        osDelay(1);
        network_set_master(network_get_ownid());

        msg_share_t share;
        uwb_build_share(&share, network_get_expected_seq_num(), sleep_time);

        msg_t tx_msg = {
            .type       = MSG_TYPE_SHARE,
            .sender     = network_get_ownid(),
            .receiver   = ALL_ID,
            .data.share = share,
        };

        dwm_tx_frame_t tx_frame = msg_encode(&tx_msg);

        if (dwm_tx(&tx_frame) != DWM_TX_OK) {
            if (dwm_tx(&tx_frame) != DWM_TX_OK) {
                mprintf("ERROR: SHARE TX failed\r\n");
                return 0;
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
        uint32_t t_start  = osKernelGetTickCount();
        bool     first_rx = true;

        while (1) {
            uint32_t elapsed = osKernelGetTickCount() - t_start;
            if (elapsed >= T_SHARE_RX_WAIT) {
                mprintf("ERROR: SHARE timeout\r\n");
                return 0;
            }

            dwm_rx(&rx_frame, T_SHARE_RX_WAIT - elapsed, true, first_rx);
            first_rx = (rx_frame.type != DWM_RX_OK);

            switch (rx_frame.type) {
            case DWM_RX_OK: {
                msg_t rx_msg;
                msg_decode(&rx_frame, &rx_msg);

                if (rx_msg.type == MSG_TYPE_SHARE) {
                    if (!seq_ok(rx_msg.data.share.seq_num, "SHARE"))
                        break;
                    network_set_master(rx_msg.sender);
                    /* ── Apply broadcast data into network state ── */
                    uwb_read_share(&rx_msg.data.share);

                    uint32_t adjusted = rx_msg.data.share.sleep_time - T_EARLY_WKUP;
                    mprintf("[SHARE] received sleep=%lu ms nodes=%u\r\n",
                            adjusted, rx_msg.data.share.node_count);
                    return adjusted;
                }
                mprintf("ERROR: unexpected msg 0x%02X during SHARE wait\r\n",
                        rx_msg.type);
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

    msg_t tx_msg = {
        .type     = MSG_TYPE_SYNC,
        .sender   = network_get_ownid(),
        .receiver = sync_rx->sender,
        .data.sync = {
            .seq_num    = sync_rx->data.sync.seq_num,
            .peer_count = 0,
        },
    };

    dwm_tx_frame_t tx_frame = msg_encode(&tx_msg);

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
        .type     = MSG_TYPE_SYNC,
        .sender   = network_get_ownid(),
        .receiver = ALL_ID,
        .data.sync = sync_msg,
    };

    dwm_tx_frame_t tx_frame = msg_encode(&tx_msg);

    if (dwm_tx(&tx_frame) != DWM_TX_OK) {
        if (dwm_tx(&tx_frame) != DWM_TX_OK) {
            mprintf("ERROR: SYNC TX failed\r\n");
            return false;
        }
    }

    mprintf("[SYNC] sent seq=%d peers=%d\r\n", sync_msg.seq_num, sync_msg.peer_count);
    return true;
}

static bool uwb_send_POLL(uint8_t seq_num, uint16_t target_id, uint64_t *out_poll_tx)
{
    msg_t tx_msg = {
        .type     = MSG_TYPE_POLL,
        .sender   = network_get_ownid(),
        .receiver = target_id,
        .data.poll = { .seq_num = seq_num },
    };

    dwm_tx_frame_t tx_frame = msg_encode(&tx_msg);

    if (dwm_tx(&tx_frame) != DWM_TX_OK) {
        if (dwm_tx(&tx_frame) != DWM_TX_OK) {
            mprintf("ERROR: POLL TX failed\r\n");
            return false;
        }
    }

    *out_poll_tx = tx_frame.tx_timestamp;
    return true;
}

static bool uwb_send_RESPONSE(uint8_t seq_num, uint16_t target_id, uint64_t *out_resp_tx)
{
    msg_t tx_msg = {
        .type     = MSG_TYPE_RESPONSE,
        .sender   = network_get_ownid(),
        .receiver = target_id,
        .data.response = { .seq_num = seq_num },
    };

    dwm_tx_frame_t tx_frame = msg_encode(&tx_msg);

    if (dwm_tx(&tx_frame) != DWM_TX_OK) {
        if (dwm_tx(&tx_frame) != DWM_TX_OK) {
            mprintf("ERROR: RESPONSE TX failed\r\n");
            return false;
        }
    }

    *out_resp_tx = tx_frame.tx_timestamp;
    return true;
}

static bool uwb_send_FINAL(uint8_t seq_num, uint16_t target_id, msg_final_t *final_msg)
{

    uint32_t flags = osThreadFlagsWait(0x02, osFlagsWaitAll, 2);
    float pitch_rad = 0, speed_horiz = 0, vel_z = 0;
    if (!(flags & 0x80000000U) && (flags & 0x02))
        imu_get_results(&pitch_rad, &speed_horiz, &vel_z);

    final_msg->IMU_pitch_rad = pitch_rad;
    final_msg->IMU_vel_horiz = speed_horiz;
    final_msg->IMU_vel_vert  = vel_z;

    uint32_t now_hi32       = dwt_readsystimestamphi32();
    uint64_t now            = (uint64_t)now_hi32 << 8;
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

    final_msg->final_tx_ts = final_tx_exact;

    msg_t tx_msg = {
        .type     = MSG_TYPE_FINAL,
        .sender   = network_get_ownid(),
        .receiver = target_id,
        .data.final = *final_msg,
    };

    dwm_tx_frame_t tx_frame = msg_encode(&tx_msg);

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

static uwb_etwr_result_t uwb_wait_for_PASSIVES(uint64_t entries[],
                                               int16_t entry_pwr_diff_q8[],
                                               uint16_t entry_ids[],
                                               uint8_t *out_count,
                                               msg_passive_t out_passive_msgs[],
                                               uint16_t target_id)
{
    uint8_t peer_count = 0;
    const node_t *peers = network_get_peers(&peer_count);

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

    mprintf("[PASSIVE] expecting %d\r\n", expected_count);

    dwm_rx_flush();
    dwm_rx_frame_t rx_frame = {0};
    uint32_t t_start   = osKernelGetTickCount();
    uint8_t  idx       = 0;
    uint32_t wait_time = T_PASSIVE_TX_WAIT_MS * expected_count;
    bool first_rx = true;
    while (1) {
        uint32_t elapsed = osKernelGetTickCount() - t_start;
        if (elapsed >= wait_time) {
            remove_silent_passive_peers(expected_ids, expected_count, entry_ids, idx);
            *out_count = idx;
            if (idx < expected_count)
                mprintf("ERROR: PASSIVE timeout (%d/%d received)\r\n", idx, expected_count);
            return UWB_TWR_TIMEOUT;
        }

        dwm_rx(&rx_frame, wait_time - elapsed, true, first_rx);
        first_rx = (rx_frame.type != DWM_RX_OK);

        switch (rx_frame.type) {
        case DWM_RX_OK: {
            msg_t rx_msg;
            msg_decode(&rx_frame, &rx_msg);

            if (rx_msg.type == MSG_TYPE_PASSIVE) {
                if (!seq_ok(rx_msg.data.passive.seq_num, "PASSIVE"))
                    break;
                if (rx_msg.receiver != network_get_master()) {
                    mprintf("ERROR: PASSIVE wrong receiver 0x%04X\r\n", rx_msg.receiver);
                    *out_count = idx;
                    return UWB_TWR_UNEXPECTED_MASTER;
                }

                entries[idx]          = rx_frame.rx_timestamp;
                entry_pwr_diff_q8[idx] = rx_frame.rssi_q8 - rx_frame.fp_q8;
                out_passive_msgs[idx] = rx_msg.data.passive;
                entry_ids[idx]        = rx_msg.sender;
                idx++;

                if (idx == expected_count) {
                    mprintf("[PASSIVE] all %d received\r\n", expected_count);
                    *out_count = idx;
                    return UWB_TWR_RECEIVED;
                }
                //osDelay(2);
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
            return UWB_TWR_TIMEOUT;
        }
    }
}

static void remove_silent_passive_peers(const uint16_t *expected_ids,
                                        uint8_t expected_count,
                                        const uint16_t *heard_ids,
                                        uint8_t heard_count)
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

static uwb_etwr_result_t uwb_send_PASSIVE(uint16_t initiator_id,
                                          uint16_t responder_id,
                                          msg_passive_t *passive_msg)
{
    /* Compute own TX-order index without disturbing network state */
    int8_t my_idx = 0;
    uint8_t peer_count = 0;
    const node_t *peers = network_get_peers(&peer_count);
    for (uint8_t i = 0; i < peer_count; i++) {
        if (peers[i].id == initiator_id) continue;
        if (peers[i].id == responder_id) continue;
        if (peers[i].id == network_get_ownid()) break;
        my_idx++;
    }

    dwm_rx_flush();
    dwm_rx_frame_t rx_frame = {0};
    uint32_t t_start   = osKernelGetTickCount();
    uint8_t  idx       = 0;
    uint32_t wait_time = T_PASSIVE_TX_WAIT_MS * my_idx;
    bool first_rx = true;
    while (idx < my_idx) {
        uint32_t elapsed = osKernelGetTickCount() - t_start;
        if (elapsed >= wait_time) {
            mprintf("ERROR: PASSIVE pre-TX wait timeout (got %d/%d)\r\n", idx, my_idx);
            return UWB_TWR_TIMEOUT;
        }

        dwm_rx(&rx_frame, wait_time - elapsed, true, first_rx);
        first_rx = (rx_frame.type != DWM_RX_OK);

        switch (rx_frame.type) {
        case DWM_RX_OK: {
            msg_t rx_msg;
            msg_decode(&rx_frame, &rx_msg);

            if (rx_msg.type == MSG_TYPE_PASSIVE) {
                if (rx_msg.receiver != network_get_master()) {
                    mprintf("ERROR: PASSIVE wrong master 0x%04X\r\n", rx_msg.receiver);
                    return UWB_TWR_UNEXPECTED_MASTER;
                }
                if (!seq_ok(rx_msg.data.passive.seq_num, "PASSIVE"))
                    break;

                passive_msg->entries[idx]       = rx_frame.rx_timestamp;
                passive_msg->entry_pwr_diff_q8[idx] = rx_frame.rssi_q8 - rx_frame.fp_q8;
                passive_msg->entry_ids[idx]     = rx_msg.sender;
                idx++;
            }
            break;
        }
        case DWM_RX_ERR:
            mprintf("ERROR: PASSIVE RX error 0x%08lX\r\n", rx_frame.status);
            break;
        case DWM_RX_TIMEOUT:
            mprintf("ERROR: PASSIVE pre-TX wait timeout (got %d/%d)\r\n", idx, my_idx);
            return UWB_TWR_TIMEOUT;
        }
    }

    passive_msg->entry_count = idx;

    uint32_t flags = osThreadFlagsWait(0x02, osFlagsWaitAll, 2);
    float pitch_rad = 0, speed_horiz = 0, vel_z = 0;
    if (!(flags & 0x80000000U) && (flags & 0x02))
        imu_get_results(&pitch_rad, &speed_horiz, &vel_z);

    passive_msg->IMU_pitch_rad = pitch_rad;
    passive_msg->IMU_vel_horiz = speed_horiz;
    passive_msg->IMU_vel_vert  = vel_z;

    /* Schedule TX — 9-bit aligned delayed TX to embed correct timestamp. */
    uint32_t now_hi32           = dwt_readsystimestamphi32();
    uint64_t now                = (uint64_t)now_hi32 << 8;
    uint64_t passive_tx_desired = now + T_PASSIVE_TX_ASAP_TICKS;

    uint16_t tx_ant_delay  = dwt_gettxantennadelay();
    uint64_t raw           = (passive_tx_desired - tx_ant_delay) & 0xFFFFFFFFFFULL;
    uint64_t raw_aligned   = raw & 0xFFFFFFFE00ULL;
    uint64_t passive_tx_exact = raw_aligned + tx_ant_delay;

    passive_msg->passive_tx_ts = passive_tx_exact;

    msg_t tx_msg = {
        .type     = MSG_TYPE_PASSIVE,
        .sender   = network_get_ownid(),
        .receiver = network_get_master(),
        .data.passive = *passive_msg,
    };

    dwm_tx_frame_t tx_frame = msg_encode(&tx_msg);

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
