/**
 * @file uwb_network.c
 * @brief Implementation of the UWB network state management API.
 *
 * Maintains a single static @ref network_t instance (@c net) that holds
 * peer state, master election state, and all measurement data for the
 * current DS-TWR exchange. All public functions operate on this instance.
 */

#include "cmsis_os.h"
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include "cmsis_os2.h"

#include "../Generic/my_print.h"
#include "DWM3000_setup.h"
#include "DWM3000_driver.h"
#include "../UWB_app/uwb_network.h"
#include "../Calculations/distance.h"

static network_t net;

/* -----------------------------------------------------------------------
 * Internal helpers
 * ----------------------------------------------------------------------- */

/**
 * @brief Look up a peer by network ID.
 *
 * Linear scan of the peer array. O(n) where n <= @ref NETWORK_MAX_PEERS.
 *
 * @param id Network ID to find.
 * @return Pointer to the matching @ref node_t, or @c NULL if not found.
 */
node_t *find_peer(uint16_t id)
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
    net.self.id = own_id;
    net.acknowledged = false;

    for (uint8_t i = 0; i < NETWORK_MAX_PEERS; i++)
        net.self.peers[i].k = 1.0;
}

/* -----------------------------------------------------------------------
 * Peer management
 * ----------------------------------------------------------------------- */

void network_add_peer(uint16_t id)
{
    if (find_peer(id) != NULL) return;
    if (net.count >= NETWORK_MAX_PEERS) return;

    node_t *p = &net.peers[net.count];
    memset(p, 0, sizeof(node_t));
    p->id = id;

    /* Seed k = 1.0 (uninitialised sentinel) for all peer slots */
    for (uint8_t i = 0; i < NETWORK_MAX_PEERS; i++)
        p->peers[i].k = 1.0;

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

            /* Reset self's slot for this peer so it gets highest priority on rejoin */
            for (uint8_t j = 0; j < NETWORK_MAX_PEERS; j++) {
                if (net.self.peers[j].peer_id == id) {
                    net.self.peers[j].certainty      = 0;
                    net.self.peers[j].distance_scaled = 0xFFFF;
                    net.self.peers[j].k              = 1.0;
                    break;
                }
            }
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
            for (uint8_t j = 0; j < NETWORK_MAX_PEERS; j++)
                new_peers[new_count].peers[j].k = 1.0;
            new_count++;
        }
    }

    /* Add all peers from SYNC, preserving existing data if known */
    for (int i = 0; i < peer_count && new_count < NETWORK_MAX_PEERS; i++) {
        node_t *existing = find_peer(peer_ids[i]);
        if (existing) {
            new_peers[new_count++] = *existing;
        } else {
            new_peers[new_count].id = peer_ids[i];
            for (uint8_t j = 0; j < NETWORK_MAX_PEERS; j++)
                new_peers[new_count].peers[j].k = 1.0;
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
    if (id == net.self.id) {
        network_remove_peer(net.self.id);  // master never sends PASSIVE — self not needed in list
    }
}

uint16_t network_get_master      (void)         { return net.master_id;                        }
bool     network_is_master       (void)         { return net.master_id == net.self.id;         }
uint16_t network_get_ownid       (void)         { return net.self.id;                          }
uint8_t  network_get_count       (void)         { return net.count;                            }
void     network_set_acknowledged(bool s)       { net.acknowledged = s;                        }
bool     network_is_acknowledged (void)         { return net.acknowledged;                     }

/* -----------------------------------------------------------------------
 * Position and uncertainty
 * ----------------------------------------------------------------------- */

void network_set_self_pos(const float pos[3])
{
    net.self.pos[0] = pos[0];
    net.self.pos[1] = pos[1];
    net.self.pos[2] = pos[2];
}

uint16_t network_get_highest_uncertainty(void)
{
    uint16_t target_id = 0;
    uint8_t  lowest    = 255;

    for (int i = 0; i < net.count; i++) {
        if (net.peers[i].id == net.self.id) continue;

        uint16_t pid = net.peers[i].id;

        /* Find self's ranging slot for this peer */
        bool found = false;
        for (uint8_t j = 0; j < NETWORK_MAX_PEERS; j++) {
            if (net.self.peers[j].peer_id != pid) continue;
            found = true;
            if (net.self.peers[j].certainty < lowest) {
                lowest    = net.self.peers[j].certainty;
                target_id = pid;
            }
            break;
        }

        /* Never ranged with this peer — highest priority */
        if (!found) return pid;
    }
    return target_id;
}

/* -----------------------------------------------------------------------
 * Peer index lookup
 * ----------------------------------------------------------------------- */

int8_t network_get_peer_index(uint16_t id)
{
    for (int8_t i = 0; i < net.count; i++)
        if (net.peers[i].id == id) return i;
    return -1;
}

/* -----------------------------------------------------------------------
 * Measurements
 * ----------------------------------------------------------------------- */

void network_reset_measurements(void)
{
    memset(&net.measurements, 0, sizeof(measurements_t));
}

/* ---- POLL ---- */

void network_store_poll(const msg_poll_t *msg, const uwb_rx_meas_t *rx)
{
    net.measurements.poll    = *msg;
    net.measurements.poll_rx = *rx;
}

/* ---- RESPONSE TX (responder's own outgoing timestamp) ---- */

void network_store_resp_tx(uint64_t ts)
{
    net.measurements.resp_tx = ts;
}

/* ---- FINAL ---- */

void network_store_final(const msg_final_t *msg, const uwb_rx_meas_t *rx)
{
    net.measurements.final    = *msg;
    net.measurements.final_rx = *rx;
}

/* ---- PASSIVE (one report per call, index = arrival order) ---- */

bool network_store_passive(uint8_t index, const msg_passive_t *msg,
                           const uwb_rx_meas_t *rx, uint16_t device_id)
{
    if (index >= NETWORK_MAX_PEERS - 2) return false;
    net.measurements.passive[index]    = *msg;
    net.measurements.passive_rx[index] = *rx;
    net.measurements.passive_count     = index + 1;
    net.measurements.passive_device_id[index] = device_id;
    return true;
}


/* ---- Read ---- */

const measurements_t *network_get_measurements(void)
{
    return &net.measurements;
}

uint8_t network_get_passive_count(void)
{
    return net.measurements.passive_count;
}

/* -----------------------------------------------------------------------
 * Sequence number
 * ----------------------------------------------------------------------- */

void    network_set_expected_seq_num(uint8_t seq_num) { net.expected_seq_num = seq_num; }
uint8_t network_get_expected_seq_num(void)            { return net.expected_seq_num;    }

/* -----------------------------------------------------------------------
 * Per-peer ranging state (distance, k, certainty)
 * ----------------------------------------------------------------------- */

static node_peer_state_t *find_or_add_peer_slot(node_t *owner, uint16_t peer_id)
{
    for (uint8_t i = 0; i < NETWORK_MAX_PEERS; i++)
        if (owner->peers[i].peer_id == peer_id) return &owner->peers[i];
    for (uint8_t i = 0; i < NETWORK_MAX_PEERS; i++) {
        if (owner->peers[i].peer_id == 0) {
            owner->peers[i].peer_id = peer_id;
            owner->peers[i].k       = 1.0;
            return &owner->peers[i];
        }
    }
    mprintf("[ERR] network: no free peer slot in node %u for %u\n", owner->id, peer_id);
    return NULL;
}

node_peer_state_t *network_get_peer_state(uint16_t owner_id, uint16_t peer_id)
{
    node_t *owner = (owner_id == net.self.id) ? &net.self : find_peer(owner_id);
    if (!owner) return NULL;
    return find_or_add_peer_slot(owner, peer_id);
}

void network_decay_certainty(void)
{
    for (int i = 0; i < net.count; i++)
        for (uint8_t j = 0; j < NETWORK_MAX_PEERS; j++) {
            if (net.peers[i].peers[j].peer_id == 0) continue;
            if (net.peers[i].peers[j].certainty >= 2)
                net.peers[i].peers[j].certainty -= 2;
            else
                net.peers[i].peers[j].certainty = 0;
        }
    for (uint8_t j = 0; j < NETWORK_MAX_PEERS; j++) {
        if (net.self.peers[j].peer_id == 0) continue;
        if (net.self.peers[j].certainty >= 2)
            net.self.peers[j].certainty -= 2;
        else
            net.self.peers[j].certainty = 0;
    }
}

void network_update_certainty(uint16_t a, uint16_t b, bool ds_twr)
{
    node_peer_state_t *s = network_get_peer_state(a, b);
    if (!s) return;
    uint8_t inc = ds_twr ? 3 : 1;
    if (s->certainty + inc > 255) s->certainty = 255;
    else s->certainty += inc;
}

void network_print_certainty(void)
{
    mprintf("[CERT] self=0x%04X peers=%u\n", net.self.id, net.count);

    /* Self's peer slots — these are what network_get_highest_uncertainty() reads */
    for (uint8_t j = 0; j < NETWORK_MAX_PEERS; j++) {
        if (net.self.peers[j].peer_id == 0) continue;
        mprintf("[CERT]   self->0x%04X  certainty=%u  dist=%.1f ticks\n",
                net.self.peers[j].peer_id,
                net.self.peers[j].certainty,
                dist_scale_to_ticks(net.self.peers[j].distance_scaled));
    }

    /* Each peer's internal slots (decayed by network_decay_certainty) */
    for (int i = 0; i < net.count; i++) {
        for (uint8_t j = 0; j < NETWORK_MAX_PEERS; j++) {
            if (net.peers[i].peers[j].peer_id == 0) continue;
            mprintf("[CERT]   0x%04X->0x%04X  certainty=%u  dist=%.1f ticks\n",
                    net.peers[i].id,
                    net.peers[i].peers[j].peer_id,
                    net.peers[i].peers[j].certainty,
                    dist_scale_to_ticks(net.peers[i].peers[j].distance_scaled));
        }
    }
}




void network_set_distance(uint16_t a, uint16_t b, uint16_t dist_scaled) { 
    node_peer_state_t *s = network_get_peer_state(a,b); 
    if(s) s->distance_scaled = dist_scaled; 
}

uint16_t network_get_distance(uint16_t a, uint16_t b) { 
    node_peer_state_t *s = network_get_peer_state(a,b); 
    return s ? s->distance_scaled : 0xFFFF; 
}

void    network_set_k          (uint16_t a, uint16_t b, double k)  { node_peer_state_t *s = network_get_peer_state(a,b); if(s) s->k = k; }
double  network_get_k         (uint16_t a, uint16_t b)            { node_peer_state_t *s = network_get_peer_state(a,b); return s ? s->k : 1.0; }
void    network_bump_certainty (uint16_t a, uint16_t b)           { node_peer_state_t *s = network_get_peer_state(a,b); if(s && s->certainty < 255) s->certainty++; }
void    network_reset_certainty(uint16_t a, uint16_t b)           { node_peer_state_t *s = network_get_peer_state(a,b); if(s) s->certainty = 0; }
uint8_t network_get_certainty  (uint16_t a, uint16_t b)           { node_peer_state_t *s = network_get_peer_state(a,b); return s ? s->certainty : 0; }