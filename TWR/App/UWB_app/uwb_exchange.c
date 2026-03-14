#include "cmsis_os.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "cmsis_os2.h"

#include "../Generic/my_print.h"
#include "DWM3000_setup.h"
#include "DWM3000_driver.h"

#include "messages.h"
#include "uwb_network.h"
#include "uwb_exchange.h"


/* Forward declarations */
static bool uwb_send_sync_reply(const msg_t *sync_rx);
static bool uwb_sync_to_allid(uint8_t seq_num);

/* -----------------------------------------------------------------------
 * Internal helpers
 * --------------------------------------------------------------------- */

/**
 * @brief Blocking delay derived from device ID.
 *
 * Spreads transmit attempts across a ~10 ms window so that devices
 * do not all reply simultaneously.  The lower byte of the ID is XOR-folded
 * with the upper byte to improve distribution when devices share a common
 * lower byte (e.g. 0x1042 and 0x2042).
 *
 * Delay range: 0 – 9 ms (1 ms resolution via osDelay).
 *
 * @param device_id  This device's 16-bit network ID.
 */
void uwb_id_delay(uint16_t device_id)
{
    uint32_t delay_ms = (device_id % 256) * 10 / 256;
    osDelay(delay_ms);
}

/* TODO: Sub-ms resolution — replace osDelay with hardware TIM counter.
 *
 * void delay_us(uint32_t us)
 * {
 *     __HAL_TIM_SET_COUNTER(&htim2, 0);
 *     while (__HAL_TIM_GET_COUNTER(&htim2) < us);
 * }
 *
 * Then: delay_us(bucket * 10000U / 256U);
 */

/**
 * @brief Check whether a given ID is listed in a SYNC message's peer list.
 *
 * @param sync  Pointer to the received sync payload.
 * @param id    Device ID to search for.
 * @return      true if @p id is found, false otherwise.
 */
static bool sync_peer_contains(const msg_sync_t *sync, uint16_t id)
{
    for (int i = 0; i < sync->peer_count; i++) {
        if (sync->peer_ids[i] == id) return true;
    }
    return false;
}

/* -----------------------------------------------------------------------
 * Public API
 * --------------------------------------------------------------------- */

/**
 * @brief Perform one UWB synchronisation iteration.
 *
 * Behaviour depends on the current network role:
 *
 * **Master:**
 *  1. Broadcasts a SYNC frame carrying the list of all known active peers.
 *  2. Listens for replies during a @c T_SYNC_RX_ANSWER ms window.
 *  3. On the first valid reply, adds the sender as a new peer and returns
 *     @c UWB_SYNC_MASTER.  Only one new peer is acknowledged per call —
 *     the caller is expected to call this function repeatedly until all
 *     peers are acknowledged.
 *  4. On dual-master detection the higher ID wins; the loser yields,
 *     demotes itself, and re-enters as a slave (one level of recursion).
 *
 * **Slave:**
 *  1. Listens indefinitely for a SYNC broadcast.
 *  2. If own ID is already in the peer list the device returns
 *     @c UWB_SYNC_SLAVE_ACKNOWLEDGED.
 *  3. If own ID is not in the list the device sends a join reply and
 *     returns @c UWB_SYNC_SLAVE_PENDING.  The caller should invoke
 *     this function again on the next sync cycle.
 *  4. On two consecutive timeouts the device promotes itself to master
 *     and re-enters as master (one level of recursion only), returning
 *     the result of that master call.
 *
 * @note The DWM3000 must be awake before calling this function.
 * @note TODO: Add promote_attempts guard against recursive promotion cycle.
 *       Unlikely in small networks but possible when multiple devices
 *       power on simultaneously with similar IDs.
 *
 * @param seq_num  Sequence number to embed in the outgoing SYNC frame.
 *
 * @retval UWB_SYNC_MASTER              Master SYNC sent and one peer acknowledged.
 * @retval UWB_SYNC_MASTER_NO_REPLY     Master SYNC sent but no replies received in window.
 * @retval UWB_SYNC_SLAVE_ACKNOWLEDGED  Own ID confirmed in master's peer list.
 * @retval UWB_SYNC_SLAVE_PENDING       Join reply sent, awaiting acknowledgement next cycle.
 * @retval UWB_SYNC_SLAVE_REPLY_FAILED  Join reply TX failed twice, will retry next cycle.
 * @retval UWB_SYNC_NO_MASTER           Timed out, promoted self to master (promotion failed).
 * @retval UWB_SYNC_TX_FAILED           Could not transmit SYNC frame — hardware problem.
 */
uwb_sync_result_t uwb_sync(uint8_t seq_num)
{
    if (network_is_master()) {

        /* ---- MASTER: broadcast SYNC then collect one reply ---- */

        if (!uwb_sync_to_allid(seq_num)) {
            mprintf("UWB_sync [MASTER] - Failed to send SYNC, aborting\r\n");
            return UWB_SYNC_TX_FAILED;
        }

        dwm_rx_frame_t rx_frame = {0};
        uint32_t t_start = osKernelGetTickCount();

        /* Listen until T_SYNC_RX_ANSWER ms have elapsed */
        while ((osKernelGetTickCount() - t_start) < T_SYNC_RX_ANSWER) {
            dwm_rx(&rx_frame, T_SYNC_RX_ANSWER, true);

            switch (rx_frame.type) {

            case DWM_RX_OK:
                msg_t rx_msg;
                msg_decode(&rx_frame, &rx_msg);

                if (rx_msg.type == MSG_TYPE_SYNC) {

                    if (rx_msg.receiver == network_get_ownid()) {
                        /* Valid join reply — acknowledge one new peer per cycle */
                        network_add_peer(rx_msg.sender);
                        mprintf("UWB_sync [MASTER] - Peer 0x%04X acknowledged\r\n", rx_msg.sender);
                        return UWB_SYNC_MASTER;

                    } else {
                        /* SYNC from another master — dual master conflict.
                         * Resolution: higher ID wins.
                         * The loser demotes itself and re-enters as slave. */
                        mprintf("UWB_sync [MASTER] - Dual master detected (sender: 0x%04X)\r\n", rx_msg.sender);

                        if (rx_msg.sender > network_get_ownid()) {
                            network_set_master(rx_msg.sender);
                            mprintf("UWB_sync [MASTER] - Yielding to 0x%04X\r\n", rx_msg.sender);
                            return uwb_sync(seq_num); /* re-enter as slave, one level deep */
                        }
                        /* Own ID is higher — other master should yield on its next cycle */
                        mprintf("UWB_sync [MASTER] - Other master 0x%04X should yield\r\n", rx_msg.sender);
                    }

                } else {
                    /* Non-SYNC traffic during the answer window — ignore */
                    mprintf("UWB_sync [MASTER] - Unexpected message type 0x%02X, ignoring\r\n", rx_msg.type);
                }
                break;

            case DWM_RX_ERR:
                mprintf("UWB_sync [MASTER] - RX error: 0x%08lX\r\n", rx_frame.status);
                break;

            case DWM_RX_TIMEOUT:
                /* Inner timeout — no reply arrived before dwm_rx gave up.
                 * The outer while loop will exit if the wall-clock window
                 * has also elapsed. */
                mprintf("UWB_sync [MASTER] - SYNC window closed\r\n");
                return UWB_SYNC_MASTER_NO_REPLY;
            }
        }

        mprintf("UWB_sync [MASTER] - SYNC window closed (time elapsed)\r\n");
        return UWB_SYNC_MASTER_NO_REPLY;

    } else {

        /* ---- SLAVE: wait for SYNC and attempt to join ---- */

        dwm_rx_frame_t rx_frame  = {0};
        uint8_t        timeout_count = 0;

        while (1) {
            dwm_rx(&rx_frame, T_SYNC_RX_SET, true);

            switch (rx_frame.type) {

            case DWM_RX_OK:
                msg_t rx_msg;
                msg_decode(&rx_frame, &rx_msg);

                switch (rx_msg.type) {

                case MSG_TYPE_SYNC:
                    if (rx_msg.receiver == ALL_ID) {

                        /* Validate sender — reject rogue masters with lower ID */
                        if (rx_msg.sender != network_get_master()) {
                            mprintf("UWB_sync [SLAVE] - Unexpected master: 0x%04X, expected: 0x%04X\r\n",
                                    rx_msg.sender, network_get_master());

                            if (rx_msg.sender > network_get_master()) {
                                /* Higher-ID master takes over */
                                network_set_master(rx_msg.sender);
                                mprintf("UWB_sync [SLAVE] - New master: 0x%04X\r\n", rx_msg.sender);
                                network_update_peers_from_sync(
                                rx_msg.sender,
                                rx_msg.data.sync.peer_ids,
                                rx_msg.data.sync.peer_count);
                            
                            } else {
                                /* Rogue lower-ID master — discard frame */
                                break;
                            }
                        } else {
                            /* Known master — refresh peer list */
                            network_update_peers_from_sync(
                                rx_msg.sender,
                                rx_msg.data.sync.peer_ids,
                                rx_msg.data.sync.peer_count);
                        }

                        if (sync_peer_contains(&rx_msg.data.sync, network_get_ownid())) {
                            /* Own ID present in peer list — device is acknowledged */
                            mprintf("UWB_sync [SLAVE] - Acknowledged by master 0x%04X\r\n", rx_msg.sender);
                            return UWB_SYNC_SLAVE_ACKNOWLEDGED;
                        } else {
                            /* Not yet in the network — send a join reply */
                            mprintf("UWB_sync [SLAVE] - Not acknowledged, sending reply\r\n");
                            if (!uwb_send_sync_reply(&rx_msg)) {
                                /* Reply failed twice — will retry on next SYNC */
                                mprintf("UWB_sync [SLAVE] - SYNC reply failed, will retry\r\n");
                                return UWB_SYNC_SLAVE_REPLY_FAILED;
                            }
                            return UWB_SYNC_SLAVE_PENDING;
                        }

                    } else {
                        /* SYNC addressed directly to the master — another slave
                         * is replying.  Wait to avoid colliding with it. */
                        mprintf("UWB_sync [SLAVE] - Detected SYNC reply, waiting SYNC_TO_SYNC\r\n");
                        osDelay(SYNC_TO_SYNC);
                    }
                    break;

                case MSG_TYPE_ERR:
                    mprintf("UWB_sync [SLAVE] - Detected ERR message\r\n");
                    break;

                /* The following message types indicate an active TWR exchange.
                 * Wait for the exchange to finish before attempting to transmit
                 * so this device does not corrupt an ongoing ranging session.
                 * TODO: Put DWM3000 to sleep during these delays instead of
                 *       busy-waiting, to avoid spurious DWM_RX_TIMEOUT events. */
                case MSG_TYPE_POLL:
                    mprintf("UWB_sync [SLAVE] - Detected POLL, waiting POLL_TO_SYNC\r\n");
                    osDelay(POLL_TO_SYNC);
                    break;

                case MSG_TYPE_RESPONSE:
                    mprintf("UWB_sync [SLAVE] - Detected RESPONSE, waiting RESPONSE_TO_SYNC\r\n");
                    osDelay(RESPONSE_TO_SYNC);
                    break;

                case MSG_TYPE_FINAL:
                    mprintf("UWB_sync [SLAVE] - Detected FINAL, waiting FINAL_TO_SYNC\r\n");
                    osDelay(FINAL_TO_SYNC);
                    break;

                default:
                    mprintf("UWB_sync [SLAVE] - Unknown message type 0x%02X\r\n", rx_msg.type);
                    break;
                }
                break;

            case DWM_RX_ERR:
                mprintf("UWB_sync [SLAVE] - RX error: 0x%08lX\r\n", rx_frame.status);
                break;

            case DWM_RX_TIMEOUT:
                timeout_count++;
                mprintf("UWB_sync [SLAVE] - No SYNC received (timeout %d/%d)\r\n",
                        timeout_count, SYNC_TIMEOUT_MAX);

                if (timeout_count >= SYNC_TIMEOUT_MAX) {
                    /* No master heard after SYNC_TIMEOUT_MAX attempts.
                     * Stagger promotion using ID-based delay to reduce the
                     * chance of two devices promoting simultaneously. */
                    uwb_id_delay(network_get_ownid());
                    network_set_master(network_get_ownid());
                    mprintf("UWB_sync [SLAVE] - No master found, promoting to master\r\n");

                    if (network_is_master()) {
                        return uwb_sync(seq_num); /* re-enter as master, one level deep */
                    } else {
                        mprintf("UWB_sync [SLAVE] - Master promotion failed\r\n");
                    }
                    return UWB_SYNC_NO_MASTER;
                }
                break;
            }
        }
    }
}

/* -----------------------------------------------------------------------
 * Static helpers (defined after uwb_sync to keep forward decls minimal)
 * --------------------------------------------------------------------- */

/**
 * @brief Send a SYNC join reply to the master.
 *
 * Called by a slave that received a SYNC broadcast but whose ID was not
 * yet in the master's peer list.  Applies @c uwb_id_delay() before
 * transmitting to spread replies from multiple slaves across time.
 *
 * On TX failure the frame is retransmitted once.  If the retry also
 * fails the function returns false and the caller should wait for the
 * next SYNC cycle.
 *
 * @param sync_rx  The received SYNC message the reply is directed at.
 * @return         true on successful transmission, false on double failure.
 */
static bool uwb_send_sync_reply(const msg_t *sync_rx)
{
    network_set_acknowledged(false);
    uwb_id_delay(network_get_ownid());

    msg_sync_t reply_sync = {
        .seq_num    = sync_rx->data.sync.seq_num,
        .peer_count = 0,  /* Slave sends no peer info — master is the authority */
    };

    msg_t tx_msg = {
        .type      = MSG_TYPE_SYNC,
        .sender    = network_get_ownid(),
        .receiver  = sync_rx->sender,
        .data.sync = reply_sync,
    };

    dwm_tx_frame_t tx_frame = msg_encode(&tx_msg);

    if (dwm_tx(&tx_frame) != DWM_TX_OK) {
        mprintf("UWB_sync [SLAVE] - SYNC reply send failed, retrying\r\n");
        if (dwm_tx(&tx_frame) != DWM_TX_OK) {
            mprintf("UWB_sync [SLAVE] - SYNC reply retry failed\r\n");
            return false;
        }
    }
    mprintf("UWB_sync [SLAVE] - SYNC reply sent to 0x%04X\r\n", sync_rx->sender);
    return true;
}

/**
 * @brief Build and broadcast a SYNC frame to all devices (ALL_ID).
 *
 * Fills the peer list from the current network state via
 * @c network_fill_peer_ids(), which excludes own_id and inactive peers.
 * On TX failure the frame is retransmitted once.
 *
 * @param seq_num  Sequence number to embed in the SYNC payload.
 * @return         true on successful transmission, false on double failure.
 */
static bool uwb_sync_to_allid(uint8_t seq_num)
{
    msg_sync_t sync_msg = {
        .seq_num    = seq_num,
        .peer_count = 0,
        .peer_ids   = {0},
    };

    /* Populate peer list — master is the single source of truth */
    sync_msg.peer_count = network_fill_peer_ids(sync_msg.peer_ids, NETWORK_MAX_PEERS);

    msg_t tx_msg = {
        .type      = MSG_TYPE_SYNC,
        .sender    = network_get_ownid(),
        .receiver  = ALL_ID,
        .data.sync = sync_msg,
    };

    dwm_tx_frame_t tx_frame = msg_encode(&tx_msg);

    if (dwm_tx(&tx_frame) != DWM_TX_OK) {
        mprintf("UWB_sync [MASTER] - SYNC send failed, retrying\r\n");
        if (dwm_tx(&tx_frame) != DWM_TX_OK) {
            mprintf("UWB_sync [MASTER] - SYNC retry failed\r\n");
            return false;
        }
    }
    mprintf("UWB_sync [MASTER] - SYNC sent (seq=%d, peers=%d)\r\n",
            sync_msg.seq_num, sync_msg.peer_count);
    return true;
}



