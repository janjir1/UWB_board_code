#include "cmsis_os.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "cmsis_os2.h"

#include "../Generic/my_print.h"
#include "DWM3000_setup.h"
#include "DWM3000_driver.h"

#include "../UWB_app/uwb_network.h"

static network_t net;

/* -----------------------------------------------------------------------
 * Internal helpers
 * ----------------------------------------------------------------------- */

static node_t *find_peer(uint16_t id)
{
    for (int i = 0; i < net.count; i++)
        if (net.peers[i].id == id) return &net.peers[i];
    return NULL;
}

/* -----------------------------------------------------------------------
 * Initialisation
 * ----------------------------------------------------------------------- */

void network_init(uint16_t own_id)
{
    memset(&net, 0, sizeof(net));
    net.self.id      = own_id;
    net.acknowledged = false;
}

/* -----------------------------------------------------------------------
 * Peer management
 * ----------------------------------------------------------------------- */

void network_add_peer(uint16_t id)
{
    if (id == net.self.id)             return; /* never add self */
    if (find_peer(id) != NULL)         return; /* already present */
    if (net.count >= NETWORK_MAX_PEERS) return; /* list full */

    node_t *p = &net.peers[net.count];
    memset(p, 0, sizeof(node_t));
    p->id = id;
    p->uncertainty = 1.0;
    /* pos and uncertainty zero-initialised by memset:
     * uncertainty == 0 signals "no estimate yet" — scheduler will
     * prioritise this peer for ranging */

    net.count++;
}

void network_remove_peer(uint16_t id)
{
    for (int i = 0; i < net.count; i++) {
        if (net.peers[i].id == id) {
            /* Overwrite with last entry to keep array packed */
            net.peers[i] = net.peers[net.count - 1];
            memset(&net.peers[net.count - 1], 0, sizeof(node_t));
            net.count--;
            return;
        }
    }
}

const node_t *network_get_peers(uint8_t *out_count)
{
    *out_count = net.count;
    return net.peers;
}

network_t *network_get_network(void)
{
    return &net;
}

uint8_t network_fill_peer_ids(uint16_t *out_ids, uint8_t max_count)
{
    uint8_t count = 0;
    for (int i = 0; i < net.count && count < max_count; i++) {
        if (net.peers[i].id == net.self.id) continue; /* skip self */
        out_ids[count++] = net.peers[i].id;
    }
    return count;
}

void network_update_peers_from_sync(uint16_t master_id,
                                    const uint16_t *peer_ids,
                                    uint8_t peer_count)
{
    /* Rebuild the peer list from the master's SYNC payload.
     * Preserve existing position and uncertainty data for known peers.
     * Peers absent from the SYNC are considered gone. */
    node_t  new_peers[NETWORK_MAX_PEERS];
    uint8_t new_count = 0;

    memset(new_peers, 0, sizeof(new_peers));

    /* Add master as first entry — skip if master is self */
    if (master_id != net.self.id) {
        node_t *existing = find_peer(master_id);
        if (existing) {
            new_peers[new_count++] = *existing;
        } else {
            new_peers[new_count].id = master_id;
            new_count++;
        }
    }

    /* Add all peers from SYNC, preserving existing data if known */
    for (int i = 0; i < peer_count && new_count < NETWORK_MAX_PEERS; i++) {
        //if (peer_ids[i] == net.self.id) continue; /* never add self */
        //TODO i need self in the list, if it breaks something, im sorry
        //BUG If there is s bug wuth peers, its this

        node_t *existing = find_peer(peer_ids[i]);
        if (existing) {
            new_peers[new_count++] = *existing;
        } else {
            new_peers[new_count].id = peer_ids[i];
            new_count++;
        }
    }

    memcpy(net.peers, new_peers, sizeof(new_peers));
    net.count = new_count;
}

/* -----------------------------------------------------------------------
 * Master / self
 * ----------------------------------------------------------------------- */

void network_set_master(uint16_t id)
{
    net.master_id = id;
}

uint16_t network_get_master(void)
{
    return net.master_id;
}

bool network_is_master(void)
{
    return net.master_id == net.self.id;
}

uint16_t network_get_ownid(void)
{
    return net.self.id;
}

uint8_t network_get_count(void)
{
    return net.count;
}

void network_set_acknowledged(bool state)
{
    net.acknowledged = state;
}

bool network_is_acknowledged(void)
{
    return net.acknowledged;
}

/* -----------------------------------------------------------------------
 * Position and uncertainty
 * ----------------------------------------------------------------------- */

void network_set_self_pos(const float pos[3], float uncertainty)
{
    net.self.pos[0]      = pos[0];
    net.self.pos[1]      = pos[1];
    net.self.pos[2]      = pos[2];
    net.self.uncertainty = uncertainty;
}

uint16_t network_get_highest_uncertainty(void)
{
    uint16_t target_id = 0;
    float highest = -1.0f;
    for (int i = 0; i < net.count; i++) {
        if (net.peers[i].id == net.self.id) continue; /* skip self */
        if (net.peers[i].uncertainty > highest) {
            highest   = net.peers[i].uncertainty;
            target_id = net.peers[i].id;
        }
    }
    return target_id;
}





/* -----------------------------------------------------------------------
 * Measurements
 * ----------------------------------------------------------------------- */

void network_reset_measurements(void)
{
    memset(&net.measurements, 0, sizeof(measurements_t));
}

/* -----------------------------------------------------------------------
 * measurements — twr_timestamps_t (initiator / responder)
 * ----------------------------------------------------------------------- */

/* --- Initiator writes --- */

void network_set_twr_poll_tx(uint64_t ts)
{
    net.measurements.twr.poll_tx = ts;
}

void network_set_twr_resp_rx(const uwb_rx_meas_t *meas)
{
    net.measurements.twr.resp_rx = *meas;
}

void network_set_twr_final_tx(uint64_t ts)
{
    net.measurements.twr.final_tx = ts;
}

/* --- Responder writes --- */

void network_set_twr_poll_rx(const uwb_rx_meas_t *meas)
{
    net.measurements.twr.poll_rx = *meas;
}

void network_set_twr_resp_tx(uint64_t ts)
{
    net.measurements.twr.resp_tx = ts;
}

void network_set_twr_final_rx(const uwb_rx_meas_t *meas)
{
    net.measurements.twr.final_rx = *meas;
}


/* --- Read --- */

const twr_timestamps_t *network_get_twr(void)
{
    return &net.measurements.twr;
}

/* -----------------------------------------------------------------------
 * measurements — twr_observation_t (passive: own view of POLL/RESP/FINAL)
 * ----------------------------------------------------------------------- */

void network_set_obs_poll_rx(const uwb_rx_meas_t *meas)
{
    net.measurements.self_twr_observation.poll_rx = *meas;
}

void network_set_obs_resp_rx(const uwb_rx_meas_t *meas)
{
    net.measurements.self_twr_observation.resp_rx = *meas;
}

void network_set_obs_final_rx(const uwb_rx_meas_t *meas)
{
    net.measurements.self_twr_observation.final_rx = *meas;
}

const twr_observation_t *network_get_self_twr_observation(void)
{
    return &net.measurements.self_twr_observation;
}

/* -----------------------------------------------------------------------
 * measurements — passive_observation_t (passive: own view of passive reports)
 * ----------------------------------------------------------------------- */

/**
 * @brief Record one incoming passive report frame.
 *
 * Called each time a MSG_TYPE_PASSIVE arrives before this device
 * transmits its own passive report. Index matches arrival order
 * (0-based), which equals this device's position in the passive chain.
 * Returns false if the index is out of bounds.
 */
bool network_set_passive_report_rx(uint8_t index, const uint64_t meas)
{
    if (index >= NETWORK_MAX_PEERS - 2) return false;
    net.measurements.self_passive_observation.passive_rx[index] = meas;
    net.measurements.self_passive_count = index + 1;
    return true;
}

const passive_observation_t *network_get_self_passive_observation(void)
{
    return &net.measurements.self_passive_observation;
}

uint8_t network_get_self_passive_count(void)
{
    return net.measurements.self_passive_count;
}

/* -----------------------------------------------------------------------
 * measurements — Master-collected passive data
 * ----------------------------------------------------------------------- */

/**
 * @brief Record the master's own RX measurement of a passive node's PASSIVE frame.
 *
 * @param index  Passive node index (0-based, order of arrival).
 * @param meas   Master's RX measurement of that PASSIVE frame.
 * @return false if index is out of bounds.
 */
bool network_set_passive_ss_rx(uint8_t index, const uwb_rx_meas_t *meas)
{
    if (index >= NETWORK_MAX_PEERS - 2) return false;
    net.measurements.ss_twr[index].passive_rx = *meas;
    return true;
}

/**
 * @brief Record the TX timestamp embedded in a passive node's PASSIVE frame.
 *
 * @param index  Passive node index (0-based).
 * @param ts     TX timestamp as reported by the passive node.
 * @return false if index is out of bounds.
 */
bool network_set_passive_ss_tx(uint8_t index, uint64_t ts)
{
    if (index >= NETWORK_MAX_PEERS - 2) return false;
    net.measurements.ss_twr[index].passive_tx = ts;
    return true;
}

/**
 * @brief Record a passive node's observation of POLL, RESPONSE, and FINAL.
 *
 * @param index  Passive node index (0-based).
 * @param obs    Full POLL/RESP/FINAL observation decoded from the PASSIVE message.
 * @return false if index is out of bounds.
 */
bool network_set_passive_twr_observation(uint8_t index, const twr_observation_simple_t *obs)
{
    if (index >= NETWORK_MAX_PEERS - 2) return false;
    net.measurements.twr_observations[index].poll_rx  = obs->poll_rx;
    net.measurements.twr_observations[index].resp_rx  = obs->resp_rx;
    net.measurements.twr_observations[index].final_rx = obs->final_rx;
    return true;
}

/**
 * @brief Record a passive node's observation of preceding PASSIVE frames.
 *
 * @param index  Passive node index (0-based).
 * @param obs    Inter-passive RX timestamps decoded from the PASSIVE message.
 * @return false if index is out of bounds.
 */
bool network_set_passive_observation(uint8_t index, const passive_observation_t *obs)
{
    if (index >= NETWORK_MAX_PEERS - 2) return false;
    net.measurements.passive_observations[index] = *obs;
    return true;
}

/**
 * @brief Increment the passive report counter after a full PASSIVE frame is stored.
 *        Call once per received PASSIVE, after all four setters above.
 */
void network_increment_passive_count(void)
{
    if (net.measurements.passive_count < NETWORK_MAX_PEERS - 2)
        net.measurements.passive_count++;
}

uint8_t network_get_passive_count(void)
{
    return net.measurements.passive_count;
}

bool network_set_passive_device_id(uint8_t index, uint16_t id)
{
    if (index >= NETWORK_MAX_PEERS - 2) return false;
    net.measurements.passive_device_id[index] = id;
    return true;
}

int8_t network_get_peer_index(uint16_t id)
{
    for (int8_t i = 0; i < net.count; i++) {
        if (net.peers[i].id == id)
            return i;
    }
    return -1;
}

uint16_t network_get_passive_device_id(uint8_t index)
{
    if (index >= NETWORK_MAX_PEERS - 2) return 0;
    return net.measurements.passive_device_id[index];
}

uint64_t network_get_passive_ss_tx(uint8_t index)
{
    if (index >= NETWORK_MAX_PEERS - 2) return 0;
    return net.measurements.ss_twr[index].passive_tx;
}

uint64_t network_get_passive_twr_obs_poll_rx(uint8_t index)
{
    if (index >= NETWORK_MAX_PEERS - 2) return 0;
    return net.measurements.twr_observations[index].poll_rx;
}

uint64_t network_get_passive_twr_obs_resp_rx(uint8_t index)
{
    if (index >= NETWORK_MAX_PEERS - 2) return 0;
    return net.measurements.twr_observations[index].resp_rx;
}

uint64_t network_get_passive_twr_obs_final_rx(uint8_t index)
{
    if (index >= NETWORK_MAX_PEERS - 2) return 0;
    return net.measurements.twr_observations[index].final_rx;
}

uwb_rx_meas_t network_get_passive_ss_rx(uint8_t index)
{
    if (index >= NETWORK_MAX_PEERS - 2) return (uwb_rx_meas_t){0};
    return net.measurements.ss_twr[index].passive_rx;
}