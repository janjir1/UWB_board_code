#include "cmsis_os.h"
#include "cmsis_os2.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "../Generic/my_print.h"
#include "DWM3000_setup.h"
#include "DWM3000_driver.h"
#include "messages.h"
#include "../UWB_app/uwb_network.h"
#include "uwb_exchange.h"

/* -----------------------------------------------------------------------
 * Forward declarations
 * ----------------------------------------------------------------------- */
static bool              uwb_send_sync_reply(const msg_t *sync_rx);
static bool              uwb_sync_to_all_id(uint8_t seq_num);
static bool              uwb_send_POLL(uint8_t seq_num, uint16_t target_id);
static bool              uwb_send_RESPONSE(uint8_t seq_num, uint16_t target_id);
static bool              uwb_send_FINAL(uint8_t seq_num, uint16_t target_id);
static uwb_etwr_result_t uwb_tdoa_receiver(uint8_t seq_num);
static uwb_etwr_result_t uwb_tdoa_sender(uint8_t seq_num);

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
 * @brief Blocking delay derived from device ID.
 *
 * Spreads transmit attempts across a 10 ms window so that devices do not
 * all reply simultaneously. The lower byte of the ID is XOR-folded with
 * the upper byte to improve distribution when devices share a common lower
 * byte (e.g. 0x1042 and 0x2042). Delay range: 0–9 ms, 1 ms resolution.
 *
 * @param device_id This device's 16-bit network ID.
 */
static void uwb_id_delay(uint16_t device_id)
{
    uint32_t delay_ms = ((device_id & 0xFF) ^ (device_id >> 8)) % 10;
    osDelay(delay_ms);
}


/**
 * @brief Sub-microsecond busy-wait using a hardware timer.
 *
 * @param us Delay in microseconds.
 * @note TODO: verify htim2 is initialised and running before first use.
 *       Replace with a DWT cycle-counter version if htim2 is unavailable.
 */
/*
static void delay_us(uint32_t us)
{
    HAL_TIM_SET_COUNTER(&htim2, 0);
    while (HAL_TIM_GET_COUNTER(&htim2) < us);
}
*/
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
 * @param seq_num Sequence number to embed in the outgoing SYNC frame.
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
uwb_sync_result_t uwb_sync(uint8_t seq_num)
{
    if (network_is_master()) {

        /* ---- MASTER: broadcast SYNC then collect one reply ---- */
        if (!uwb_sync_to_all_id(seq_num)) {
            mprintf("UWBsync [MASTER] - Failed to send SYNC, aborting\r\n");
            return UWB_SYNC_TX_FAILED;
        }

        dwm_rx_frame_t rx_frame = {0};
        uint32_t t_start = osKernelGetTickCount();

        while (1) {
            uint32_t elapsed = osKernelGetTickCount() - t_start;
            if (elapsed >= T_SYNC_RX_ANSWER) break;

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
                    /* Valid join reply — acknowledge one new peer per cycle */
                    network_add_peer(rx_msg.sender);
                    mprintf("UWBsync [MASTER] - Peer 0x%04X acknowledged\r\n", rx_msg.sender);
                    return UWB_SYNC_MASTER;

                } else {
                    /* SYNC from another master — dual-master conflict.
                     * Higher ID wins; loser demotes and re-enters as slave. */
                    mprintf("UWBsync [MASTER] - Dual master detected sender 0x%04X\r\n",
                            rx_msg.sender);
                    if (rx_msg.sender > network_get_ownid()) {
                        network_set_master(rx_msg.sender);
                        mprintf("UWBsync [MASTER] - Yielding to 0x%04X\r\n", rx_msg.sender);
                        return UWB_SYNC_NEW_SLAVE;
                    }
                    /* Own ID is higher — other master should yield on its next cycle */
                    mprintf("UWBsync [MASTER] - Other master 0x%04X should yield\r\n",
                            rx_msg.sender);
                }
                break;
            }
            case DWM_RX_ERR:
                mprintf("UWBsync [MASTER] - RX error 0x%08lX\r\n", rx_frame.status);
                break;

            case DWM_RX_TIMEOUT:
                mprintf("UWBsync [MASTER] - SYNC window closed\r\n");
                return UWB_SYNC_MASTER_NO_REPLY;
            }
        }

        mprintf("UWBsync [MASTER] - SYNC window closed (time elapsed)\r\n");
        return UWB_SYNC_MASTER_NO_REPLY;

    } else {

        /* ---- SLAVE: wait for SYNC and attempt to join ---- */
        dwm_rx_frame_t rx_frame = {0};
        uint8_t timeout_count = 0;

        while (1) {
            /* Small yield before RX to prevent starvation of other tasks */
            osDelay(1);

            dwm_rx(&rx_frame, T_SYNC_RX_SET, true);

            switch (rx_frame.type) {
            case DWM_RX_OK: {
                msg_t rx_msg;
                msg_decode(&rx_frame, &rx_msg);

                switch (rx_msg.type) {

                case MSG_TYPE_SYNC:
                    if (rx_msg.receiver == ALL_ID) {
                        /* Validate sender — reject rogue masters with lower ID */
                        if (rx_msg.sender != network_get_master()) {
                            if (rx_msg.sender > network_get_master()) {
                                /* Higher-ID master takes over */
                                network_set_master(rx_msg.sender);
                                mprintf("UWBsync [SLAVE] - New master 0x%04X\r\n", rx_msg.sender);
                                network_update_peers_from_sync(rx_msg.sender,
                                                               rx_msg.data.sync.peer_ids,
                                                               rx_msg.data.sync.peer_count);
                            } else {
                                /* Rogue lower-ID master — discard */
                                mprintf("UWBsync [SLAVE] - Unexpected master 0x%04X, expected 0x%04X\r\n",
                                        rx_msg.sender, network_get_master());
                                break;
                            }
                        } else {
                            /* Known master — refresh peer list */
                            network_update_peers_from_sync(rx_msg.sender,
                                                           rx_msg.data.sync.peer_ids,
                                                           rx_msg.data.sync.peer_count);
                        }

                        if (sync_peer_contains(&rx_msg.data.sync, network_get_ownid())) {
                            /* Own ID present — acknowledged */
                            mprintf("UWBsync [SLAVE] - Acknowledged by master 0x%04X\r\n",
                                    rx_msg.sender);
                            network_set_acknowledged(true);
                            return UWB_SYNC_SLAVE_ACKNOWLEDGED;
                        } else {
                            network_set_acknowledged(false);
                            mprintf("UWBsync [SLAVE] - Not acknowledged, sending reply\r\n");
                            if (!uwb_send_sync_reply(&rx_msg))
                                return UWB_SYNC_SLAVE_REPLY_FAILED;
                            return UWB_SYNC_SLAVE_PENDING;
                        }

                    } else {
                        /* SYNC addressed to master — another slave is replying.
                         * Wait to avoid colliding with its transmission. */
                        mprintf("UWBsync [SLAVE] - Detected SYNC reply, waiting %d ms\r\n",
                                T_SYNC_TO_SYNC);
                        osDelay(T_SYNC_TO_SYNC);
                    }
                    break;

                case MSG_TYPE_ERR:
                    mprintf("UWBsync [SLAVE] - Detected ERR message\r\n");
                    break;

                /* The following message types indicate an active TWR exchange.
                 * Wait for the exchange to finish before transmitting, to avoid
                 * corrupting an ongoing ranging session.
                 * TODO: put DWM3000 to sleep during these delays instead of
                 *       busy-waiting, to avoid spurious DWM_RX_TIMEOUT events. */
                //TODO: put DWM3000 to sleep during these delays instead of busy-waiting, to avoid spurious DWM_RX_TIMEOUT events.
                case MSG_TYPE_POLL:
                    mprintf("UWBsync [SLAVE] - Detected POLL, waiting %d ms\r\n", T_POLL_TO_SYNC);
                    osDelay(T_POLL_TO_SYNC);
                    break;

                case MSG_TYPE_RESPONSE:
                    mprintf("UWBsync [SLAVE] - Detected RESPONSE, waiting %d ms\r\n",
                            T_RESPONSE_TO_SYNC);
                    osDelay(T_RESPONSE_TO_SYNC);
                    break;

                case MSG_TYPE_FINAL:
                    mprintf("UWBsync [SLAVE] - Detected FINAL, waiting %d ms\r\n", T_FINAL_TO_SYNC);
                    osDelay(T_FINAL_TO_SYNC);
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
                    /* No master heard — stagger promotion using ID-based delay
                     * to reduce simultaneous promotion by multiple devices. */
                    uwb_id_delay(network_get_ownid());
                    network_set_master(network_get_ownid());
                    mprintf("UWBsync [SLAVE] - No master found, promoting to master\r\n");
                    return UWB_SYNC_NEW_MASTER;
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
 * @param seq_num     Sequence number embedded in all outgoing frames.
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

// TODO: implement uwb_tdoa_receiver() and uwb_tdoa_sender() stubs.

uwb_etwr_result_t uwb_extended_twr(uint8_t seq_num, uwb_sync_result_t sync_result)
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

            if (!uwb_send_POLL(seq_num, target_id)) {
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

                        /* Save resp_rx — uwb_send_FINAL reads it via network_get_twr() */
                        network_set_twr_resp_rx(&RX_MEAS_FROM_FRAME(rx_frame));

                        if (!uwb_send_FINAL(seq_num, target_id)) {
                            mprintf("UWB_eTWR [MASTER] - Failed to send FINAL\r\n");
                            /* BUG-FIX: do NOT set master to target_id on failure —
                             * the exchange did not complete. */
                            return UWB_TWR_TX_FAILED;
                        }
                        /* final_tx timestamp saved inside uwb_send_FINAL */

                        mprintf("UWB_eTWR [MASTER] - Exchange complete with 0x%04X\r\n", target_id);
                        /* Hand master role to target — it holds all timestamps
                         * and will compute the range. */
                        network_set_master(target_id);
                        return UWB_TWR_EXCHANGE_COMPLETE;

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
                    return UWB_TWR_TIMEOUT;
                }
            }
        }

        /* ---- SLAVE: respond to or passively observe the DS-TWR exchange ---- */
        case UWB_SYNC_SLAVE_ACKNOWLEDGED:
        case UWB_SYNC_SLAVE_PENDING:
        {
            network_reset_measurements();

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
                            //no time to send
                            //mprintf("UWB_eTWR [SLAVE] - POLL received, sending RESPONSE\r\n");
                            network_set_twr_poll_rx(&RX_MEAS_FROM_FRAME(rx_frame));

                            if (!uwb_send_RESPONSE(seq_num, network_get_master())) {
                                mprintf("UWB_eTWR [SLAVE] - Failed to send RESPONSE\r\n");
                                return UWB_TWR_TX_FAILED;
                            }
                            /* resp_tx timestamp saved inside uwb_send_RESPONSE */

                        } else if (network_is_acknowledged()) {
                            /* Passive observer path */
                            mprintf("UWB_eTWR [SLAVE] - POLL observed passively (initiator: 0x%04X, target: 0x%04X)\r\n",
                                    rx_msg.sender, rx_msg.receiver);
                            network_set_obs_poll_rx(&RX_MEAS_FROM_FRAME(rx_frame));
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
                            return uwb_tdoa_receiver(seq_num);

                        } else if (network_is_acknowledged()) {
                            /* Passive observer — full set of three frames captured */
                            mprintf("UWB_eTWR [SLAVE] - FINAL observed passively\r\n");
                            network_set_obs_final_rx(&RX_MEAS_FROM_FRAME(rx_frame));
                            return uwb_tdoa_sender(seq_num);

                        } else {
                            /* Not the target and not acknowledged — uninvolved */
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
//TODO: implement ranging formula and publish result via network layer.
//TODO: apply antenna delay correction to computed distance.
static uwb_etwr_result_t uwb_tdoa_receiver(uint8_t seq_num)
{
    /* TODO: implement DS-TWR ranging formula and publish distance estimate */
    (void)seq_num;
    return UWB_TWR_RECEIVED;
}

/**
 * @brief Build and send a passive observation report (passive observer side).
 *
 * Called after a passive observer has captured POLL, RESPONSE, and FINAL
 * from an exchange it was not directly part of. Broadcasts own RX timestamps
 * so that the responder can apply TDoA corrections.
 *
 * @param seq_num Sequence number of the completed exchange.
 * @return UWB_TWR_RECEIVED_PASSIVE always (placeholder).
 */
//TODO: implement MSG_TYPE_PASSIVE frame build and transmission.
static uwb_etwr_result_t uwb_tdoa_sender(uint8_t seq_num)
{
    /* TODO: build and transmit MSG_TYPE_PASSIVE frame with own observation timestamps */
    (void)seq_num;
    return UWB_TWR_RECEIVED_PASSIVE;
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
 * @param seq_num     Forwarded to uwb_extended_twr().
 * @param sync_result Forwarded to uwb_extended_twr().

 */
 //TODO: remove or gate behind a DEBUG compile flag before production.
void uwb_twr_test(uint8_t seq_num, uwb_sync_result_t sync_result)
{
    mprintf("UWB_eTWR_TEST - Starting exchange (seq=%d, sync=%d)\r\n", seq_num, sync_result);

    uwb_etwr_result_t result = uwb_extended_twr(seq_num, sync_result);

    switch (result) {
        case UWB_TWR_EXCHANGE_COMPLETE:  mprintf("UWB_eTWR_TEST - Result: EXCHANGE_COMPLETE\r\n");  break;
        case UWB_TWR_RECEIVED:           mprintf("UWB_eTWR_TEST - Result: RECEIVED\r\n");           break;
        case UWB_TWR_RECEIVED_PASSIVE:   mprintf("UWB_eTWR_TEST - Result: RECEIVED_PASSIVE\r\n");   break;
        case UWB_TWR_NOTHING:            mprintf("UWB_eTWR_TEST - Result: NOTHING\r\n");            break;
        case UWB_TWR_TX_FAILED:          mprintf("UWB_eTWR_TEST - Result: TX_FAILED\r\n");          break;
        case UWB_TWR_TIMEOUT:            mprintf("UWB_eTWR_TEST - Result: TIMEOUT\r\n");            break;
        case UWB_TWR_UNEXPECTED_MASTER:  mprintf("UWB_eTWR_TEST - Result: UNEXPECTED_MASTER\r\n");  break;
        case UWB_TWR_NOT_ENOUGH_DEVICES: mprintf("UWB_eTWR_TEST - Result: NOT_ENOUGH_DEVICES\r\n"); break;
        default:                         mprintf("UWB_eTWR_TEST - Result: UNKNOWN (%d)\r\n", result); break;
    }

    if (result != UWB_TWR_EXCHANGE_COMPLETE &&
        result != UWB_TWR_RECEIVED         &&
        result != UWB_TWR_RECEIVED_PASSIVE) return;

    const twr_timestamps_t *twr = network_get_twr();

    mprintf("UWB_eTWR_TEST - ---- Timestamps ----\r\n");
    MPRINT_TS("poll_tx",          twr->poll_tx);
    MPRINT_TS("resp_rx.ts",       twr->resp_rx.ts);
    mprintf("UWB_eTWR_TEST - %-20s: %d (%.2f dBm)\r\n", "resp_rx.rssi_q8",
            twr->resp_rx.rssi_q8, twr->resp_rx.rssi_q8 / 256.0f);
    mprintf("UWB_eTWR_TEST - %-20s: %d (%.2f dBm)\r\n", "resp_rx.fp_q8",
            twr->resp_rx.fp_q8,   twr->resp_rx.fp_q8   / 256.0f);
    MPRINT_TS("final_tx",         twr->final_tx);
    MPRINT_TS("poll_rx.ts",       twr->poll_rx.ts);
    mprintf("UWB_eTWR_TEST - %-20s: %d (%.2f dBm)\r\n", "poll_rx.rssi_q8",
            twr->poll_rx.rssi_q8, twr->poll_rx.rssi_q8 / 256.0f);
    mprintf("UWB_eTWR_TEST - %-20s: %d (%.2f dBm)\r\n", "poll_rx.fp_q8",
            twr->poll_rx.fp_q8,   twr->poll_rx.fp_q8   / 256.0f);
    MPRINT_TS("resp_tx",          twr->resp_tx);
    MPRINT_TS("final_rx.ts",      twr->final_rx.ts);
    mprintf("UWB_eTWR_TEST - %-20s: %d (%.2f dBm)\r\n", "final_rx.rssi_q8",
            twr->final_rx.rssi_q8, twr->final_rx.rssi_q8 / 256.0f);
    mprintf("UWB_eTWR_TEST - %-20s: %d (%.2f dBm)\r\n", "final_rx.fp_q8",
            twr->final_rx.fp_q8,   twr->final_rx.fp_q8   / 256.0f);
    mprintf("UWB_eTWR_TEST - ---- Role: %s ----\r\n",
            (twr->poll_tx      != 0) ? "INITIATOR" :
            (twr->poll_rx.ts   != 0) ? "RESPONDER" : "UNKNOWN");
}

#endif
