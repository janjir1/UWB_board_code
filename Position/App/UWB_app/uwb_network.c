/**
 * @file uwb_network.c
 * @brief Implementation of the UWB network state management API.
 *
 * The active peer array is a per-round/network-snapshot view.  Its indices are
 * intentionally unstable: peers may be removed with swap-with-last packing or
 * rebuilt from SYNC.  Code that needs persistent identity must use the 16-bit
 * node ID, never a saved peer index or a saved node_t pointer.
 *
 * Distance state is also treated as a per-round snapshot.  The UWB exchange
 * calls network_reset_measurements() at the start of the round; this clears the
 * temporary exchange block and invalidates all stored distances.  The following
 * distance-calculation stage then writes only the ranges that were observed in
 * that round.  SHARE and EKF run afterwards and therefore see only the current
 * round's distances.
 */

#include "cmsis_os.h"
#include "cmsis_os2.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "../Generic/my_print.h"
#include "DWM3000_setup.h"
#include "DWM3000_driver.h"
#include "../UWB_app/uwb_network.h"
#include "../Calculations/distance.h"

#define NETWORK_DISTANCE_INVALID (0xFFFFu)

static network_t net;

/* -------------------------------------------------------------------------
 * Internal helpers
 * ------------------------------------------------------------------------- */

node_t *find_peer(uint16_t id)
{
    for (int i = 0; i < (int)net.count; i++) {
        if (net.peers[i].id == id) {
            return &net.peers[i];
        }
    }

    return NULL;
}

static bool network_id_is_active_or_self(uint16_t id)
{
    if (id == net.self.id) {
        return true;
    }

    return (find_peer(id) != NULL);
}

static node_t *network_find_owner(uint16_t owner_id)
{
    if (owner_id == net.self.id) {
        return &net.self;
    }

    return find_peer(owner_id);
}

static node_peer_state_t *find_peer_slot(node_t *owner, uint16_t peer_id)
{
    if (owner == NULL) {
        return NULL;
    }

    for (uint8_t i = 0U; i < NETWORK_MAX_PEERS; i++) {
        if (owner->peers[i].peer_id == peer_id) {
            return &owner->peers[i];
        }
    }

    return NULL;
}

static node_peer_state_t *find_or_add_peer_slot(node_t *owner, uint16_t peer_id)
{
    node_peer_state_t *slot = find_peer_slot(owner, peer_id);

    if (slot != NULL) {
        return slot;
    }

    if ((owner == NULL) || (peer_id == 0U)) {
        return NULL;
    }

    for (uint8_t i = 0U; i < NETWORK_MAX_PEERS; i++) {
        if (owner->peers[i].peer_id == 0U) {
            owner->peers[i].peer_id = peer_id;
            owner->peers[i].distance_scaled = NETWORK_DISTANCE_INVALID;
            owner->peers[i].certainty = 0U;
            return &owner->peers[i];
        }
    }

    mprintf("[ERR] network: no free peer slot in node %u for %u\n",
            owner->id,
            peer_id);
    return NULL;
}

static void invalidate_node_distances(node_t *node)
{
    if (node == NULL) {
        return;
    }

    for (uint8_t i = 0U; i < NETWORK_MAX_PEERS; i++) {
        if (node->peers[i].peer_id != 0U) {
            node->peers[i].distance_scaled = NETWORK_DISTANCE_INVALID;
        }
    }
}

static void invalidate_all_distances(void)
{
    invalidate_node_distances(&net.self);

    for (int i = 0; i < (int)net.count; i++) {
        invalidate_node_distances(&net.peers[i]);
    }
}

static void invalidate_measurements_in_node_for_id(node_t *node, uint16_t removed_id)
{
    if (node == NULL) {
        return;
    }

    for (uint8_t i = 0U; i < NETWORK_MAX_PEERS; i++) {
        if (node->peers[i].peer_id == removed_id) {
            node->peers[i].distance_scaled = NETWORK_DISTANCE_INVALID;
            node->peers[i].certainty = 0U;
            break;
        }
    }
}

static void invalidate_measurements_for_removed_id(uint16_t removed_id)
{
    invalidate_measurements_in_node_for_id(&net.self, removed_id);

    for (int i = 0; i < (int)net.count; i++) {
        invalidate_measurements_in_node_for_id(&net.peers[i], removed_id);
    }
}

static bool id_is_in_new_peers(uint16_t id, const node_t *new_peers, uint8_t new_count)
{
    if (new_peers == NULL) {
        return false;
    }

    for (uint8_t i = 0U; i < new_count; i++) {
        if (new_peers[i].id == id) {
            return true;
        }
    }

    return false;
}

/* -------------------------------------------------------------------------
 * Initialisation
 * ------------------------------------------------------------------------- */

void network_init(uint16_t own_id)
{
    memset(&net, 0, sizeof(net));
    net.self.id = own_id;
    net.acknowledged = false;
}

/* -------------------------------------------------------------------------
 * Peer management
 * ------------------------------------------------------------------------- */

void network_add_peer(uint16_t id)
{
    if (id == 0U) {
        return;
    }

    if (find_peer(id) != NULL) {
        return;
    }

    if (net.count >= NETWORK_MAX_PEERS) {
        return;
    }

    node_t *p = &net.peers[net.count];
    memset(p, 0, sizeof(node_t));
    p->id = id;
    net.count++;
}

void network_remove_peer(uint16_t id)
{
    for (int i = 0; i < (int)net.count; i++) {
        if (net.peers[i].id == id) {
            invalidate_measurements_for_removed_id(id);

            /* Overwrite with last entry to keep the active peer array packed.
             * Any saved peer index or saved node_t pointer becomes invalid. */
            net.peers[i] = net.peers[net.count - 1U];
            memset(&net.peers[net.count - 1U], 0, sizeof(node_t));
            net.count--;
            return;
        }
    }
}

const node_t *network_get_peers(uint8_t *out_count)
{
    if (out_count != NULL) {
        *out_count = net.count;
    }

    return net.peers;
}

network_t *network_get_network(void)
{
    return &net;
}

uint8_t network_fill_peer_ids(uint16_t *out_ids, uint8_t max_count)
{
    uint8_t count = 0U;

    if (out_ids == NULL) {
        return 0U;
    }

    for (int i = 0; (i < (int)net.count) && (count < max_count); i++) {
        if (net.peers[i].id == net.self.id) {
            continue;
        }

        out_ids[count] = net.peers[i].id;
        count++;
    }

    return count;
}

void network_update_peers_from_sync(uint16_t master_id,
                                    const uint16_t *peer_ids,
                                    uint8_t peer_count)
{
    node_t new_peers[NETWORK_MAX_PEERS];
    uint8_t new_count = 0U;

    memset(new_peers, 0, sizeof(new_peers));

    if ((master_id != 0U) && (master_id != net.self.id)) {
        node_t *existing = find_peer(master_id);
        if (existing != NULL) {
            new_peers[new_count] = *existing;
        } else {
            new_peers[new_count].id = master_id;
        }
        new_count++;
    }

    for (uint8_t i = 0U; (i < peer_count) && (new_count < NETWORK_MAX_PEERS); i++) {
        uint16_t pid = peer_ids[i];

        if (pid == 0U) {
            continue;
        }

        if (id_is_in_new_peers(pid, new_peers, new_count)) {
            continue;
        }

        node_t *existing = find_peer(pid);
        if (existing != NULL) {
            new_peers[new_count] = *existing;
        } else {
            new_peers[new_count].id = pid;
        }
        new_count++;
    }

    /* Invalidate ranges that refer to peers that disappeared from SYNC. */
    for (int i = 0; i < (int)net.count; i++) {
        uint16_t old_id = net.peers[i].id;
        if (!id_is_in_new_peers(old_id, new_peers, new_count)) {
            invalidate_measurements_for_removed_id(old_id);
        }
    }

    memcpy(net.peers, new_peers, sizeof(new_peers));
    net.count = new_count;

    /* SYNC starts a new active-peer snapshot.  Distance calculation will
     * repopulate only the ranges observed in the current round. */
    invalidate_all_distances();
}

/* -------------------------------------------------------------------------
 * Peer index lookup
 * ------------------------------------------------------------------------- */

int8_t network_get_peer_index(uint16_t id)
{
    for (int8_t i = 0; i < (int8_t)net.count; i++) {
        if (net.peers[i].id == id) {
            return i;
        }
    }

    return -1;
}

/* -------------------------------------------------------------------------
 * Master / self
 * ------------------------------------------------------------------------- */

void network_set_master(uint16_t id)
{
    net.master_id = id;

    if (id == net.self.id) {
        network_remove_peer(net.self.id);
    }
}

uint16_t network_get_master(void)
{
    return net.master_id;
}

bool network_is_master(void)
{
    return (net.master_id == net.self.id);
}

uint16_t network_get_ownid(void)
{
    return net.self.id;
}

uint8_t network_get_count(void)
{
    return net.count;
}

void network_set_acknowledged(bool s)
{
    net.acknowledged = s;
}

bool network_is_acknowledged(void)
{
    return net.acknowledged;
}

/* -------------------------------------------------------------------------
 * Position and uncertainty
 * ------------------------------------------------------------------------- */

void network_set_self_pos(const float pos[3])
{
    if (pos == NULL) {
        return;
    }

    net.self.pos[0] = pos[0];
    net.self.pos[1] = pos[1];
    net.self.pos[2] = pos[2];
}

/*
uint16_t network_get_highest_uncertainty(void)
{
    uint16_t target_id = 0U;
    uint16_t lowest = 256U;
    uint8_t tie_count = 0U;

    for (int i = 0; i < (int)net.count; i++) {
        uint16_t pid = net.peers[i].id;
        bool found = false;

        if (pid == net.self.id) {
            continue;
        }

        for (uint8_t j = 0U; j < NETWORK_MAX_PEERS; j++) {
            if (net.self.peers[j].peer_id != pid) {
                continue;
            }

            found = true;
            uint8_t cert = net.self.peers[j].certainty;

            if ((uint16_t)cert < lowest) {
                lowest = cert;
                target_id = pid;
                tie_count = 1U;
            } else if ((uint16_t)cert == lowest) {
                tie_count++;
                if ((rand() % (int)tie_count) == 0) {
                    target_id = pid;
                }
            }
            break;
        }

        if (!found) {
            return pid;
        }
    }

    return target_id;
}
    */

    uint16_t network_get_highest_uncertainty(void)
{
    const uint16_t PRIORITY_NODE = 0xA262U;

    uint16_t target_id = 0U;
    uint16_t lowest    = 256U;
    uint8_t  tie_count = 0U;

    /* If we ARE the priority node, fall through to normal
     * lowest-certainty logic (no special-case needed).
     * If we are NOT the priority node, check if 0xA262 is
     * reachable and return it immediately when found. */
    bool check_priority = (network_get_ownid() != PRIORITY_NODE);

    for (int i = 0; i < (int)net.count; i++) {
        uint16_t pid   = net.peers[i].id;
        bool     found = false;

        if (pid == net.self.id) {
            continue;
        }

        /* Priority-node shortcut: if 0xA262 appears in the peer
         * list and we are not that node ourselves, return it now. */
        if (check_priority && (pid == PRIORITY_NODE)) {
            return PRIORITY_NODE;
        }

        for (uint8_t j = 0U; j < NETWORK_MAX_PEERS; j++) {
            if (net.self.peers[j].peer_id != pid) {
                continue;
            }

            found = true;
            uint8_t cert = net.self.peers[j].certainty;

            if ((uint16_t)cert < lowest) {
                lowest     = cert;
                target_id  = pid;
                tie_count  = 1U;
            } else if ((uint16_t)cert == lowest) {
                tie_count++;
                if ((rand() % (int)tie_count) == 0) {
                    target_id = pid;
                }
            }
            break;
        }

        if (!found) {
            return pid;
        }
    }

    return target_id;
}

/* -------------------------------------------------------------------------
 * Measurements
 * ------------------------------------------------------------------------- */

void network_reset_measurements(void)
{
    memset(&net.measurements, 0, sizeof(measurements_t));

    /* This is the round boundary.  Keep certainty for target scheduling, but
     * invalidate all distances so SHARE and EKF cannot consume stale ranges. */
    invalidate_all_distances();
}

void network_store_poll(const msg_poll_t *msg,
                        const uwb_rx_meas_t *rx,
                        const bool antenna_unreliable)
{
    if ((msg == NULL) || (rx == NULL)) {
        return;
    }

    net.measurements.poll = *msg;
    net.measurements.poll_rx = *rx;
    net.measurements.poll_antenna_unreliable = antenna_unreliable;
}

void network_store_resp_tx(uint64_t ts)
{
    net.measurements.resp_tx = ts;
}

void network_store_final(const msg_final_t *msg,
                         const uwb_rx_meas_t *rx,
                         const bool antenna_unreliable)
{
    if ((msg == NULL) || (rx == NULL)) {
        return;
    }

    net.measurements.final = *msg;
    net.measurements.final_rx = *rx;
    net.measurements.final_antenna_unreliable = antenna_unreliable;
}

bool network_store_passive(uint8_t index,
                           const msg_passive_t *msg,
                           const uwb_rx_meas_t *rx,
                           const uint16_t device_id,
                           const bool antenna_unreliable)
{
    if ((msg == NULL) || (rx == NULL)) {
        return false;
    }

    if (index >= (uint8_t)(NETWORK_MAX_PEERS - 2U)) {
        return false;
    }

    net.measurements.passive[index] = *msg;
    net.measurements.passive_rx[index] = *rx;
    net.measurements.passive_count = (uint8_t)(index + 1U);
    net.measurements.passive_device_id[index] = device_id;
    net.measurements.passive_antenna_unreliable[index] = antenna_unreliable;

    return true;
}

const measurements_t *network_get_measurements(void)
{
    return &net.measurements;
}

uint8_t network_get_passive_count(void)
{
    return net.measurements.passive_count;
}

/* -------------------------------------------------------------------------
 * Sequence number
 * ------------------------------------------------------------------------- */

void network_set_expected_seq_num(uint8_t seq_num)
{
    net.expected_seq_num = seq_num;
}

uint8_t network_get_expected_seq_num(void)
{
    return net.expected_seq_num;
}

/* -------------------------------------------------------------------------
 * Per-peer ranging state
 * ------------------------------------------------------------------------- */

node_peer_state_t *network_get_peer_state(uint16_t owner_id, uint16_t peer_id)
{
    node_t *owner;
    node_peer_state_t *slot;

    if (!network_id_is_active_or_self(owner_id) ||
        !network_id_is_active_or_self(peer_id)) {
        return NULL;
    }

    owner = network_find_owner(owner_id);
    slot = find_peer_slot(owner, peer_id);

    if (slot == NULL) {
        return NULL;
    }

    if (slot->distance_scaled == NETWORK_DISTANCE_INVALID) {
        return NULL;
    }

    return slot;
}

void network_set_distance(uint16_t a, uint16_t b, uint16_t dist_scaled)
{
    node_t *owner;
    node_peer_state_t *slot;

    if (!network_id_is_active_or_self(a) || !network_id_is_active_or_self(b)) {
        return;
    }

    owner = network_find_owner(a);
    slot = find_or_add_peer_slot(owner, b);

    if (slot != NULL) {
        slot->distance_scaled = dist_scaled;
    }
}

uint16_t network_get_distance(uint16_t a, uint16_t b)
{
    node_t *owner;
    node_peer_state_t *slot;

    if (!network_id_is_active_or_self(a) || !network_id_is_active_or_self(b)) {
        return NETWORK_DISTANCE_INVALID;
    }

    owner = network_find_owner(a);
    slot = find_peer_slot(owner, b);

    if (slot == NULL) {
        return NETWORK_DISTANCE_INVALID;
    }

    return slot->distance_scaled;
}

void network_update_certainty(uint16_t a, uint16_t b, uint8_t certainty)
{
    node_t *owner;
    node_peer_state_t *slot;

    if (!network_id_is_active_or_self(a) || !network_id_is_active_or_self(b)) {
        return;
    }

    owner = network_find_owner(a);
    slot = find_or_add_peer_slot(owner, b);

    if (slot != NULL) {
        slot->certainty = certainty;
    }
}

void network_bump_certainty(uint16_t a, uint16_t b)
{
    node_t *owner;
    node_peer_state_t *slot;

    if (!network_id_is_active_or_self(a) || !network_id_is_active_or_self(b)) {
        return;
    }

    owner = network_find_owner(a);
    slot = find_or_add_peer_slot(owner, b);

    if ((slot != NULL) && (slot->certainty < 255U)) {
        slot->certainty++;
    }
}

void network_reset_certainty(uint16_t a, uint16_t b)
{
    node_t *owner;
    node_peer_state_t *slot;

    if (!network_id_is_active_or_self(a) || !network_id_is_active_or_self(b)) {
        return;
    }

    owner = network_find_owner(a);
    slot = find_peer_slot(owner, b);

    if (slot != NULL) {
        slot->certainty = 0U;
    }
}

uint8_t network_get_certainty(uint16_t a, uint16_t b)
{
    node_t *owner;
    node_peer_state_t *slot;

    if (!network_id_is_active_or_self(a) || !network_id_is_active_or_self(b)) {
        return 0U;
    }

    owner = network_find_owner(a);
    slot = find_peer_slot(owner, b);

    if ((slot == NULL) || (slot->distance_scaled == NETWORK_DISTANCE_INVALID)) {
        return 0U;
    }

    return slot->certainty;
}

void network_print_certainty(void)
{
    mprintf("[CERT] self=0x%04X peers=%u\n", net.self.id, net.count);

    for (uint8_t j = 0U; j < NETWORK_MAX_PEERS; j++) {
        if (net.self.peers[j].peer_id == 0U) {
            continue;
        }

        if (net.self.peers[j].distance_scaled == NETWORK_DISTANCE_INVALID) {
            mprintf("[CERT] self->0x%04X certainty=%u dist=INVALID\n",
                    net.self.peers[j].peer_id,
                    net.self.peers[j].certainty);
        } else {
            mprintf("[CERT] self->0x%04X certainty=%u dist=%.1f ticks\n",
                    net.self.peers[j].peer_id,
                    net.self.peers[j].certainty,
                    dist_scale_to_ticks(net.self.peers[j].distance_scaled));
        }
    }

    for (int i = 0; i < (int)net.count; i++) {
        for (uint8_t j = 0U; j < NETWORK_MAX_PEERS; j++) {
            if (net.peers[i].peers[j].peer_id == 0U) {
                continue;
            }

            if (net.peers[i].peers[j].distance_scaled == NETWORK_DISTANCE_INVALID) {
                mprintf("[CERT] 0x%04X->0x%04X certainty=%u dist=INVALID\n",
                        net.peers[i].id,
                        net.peers[i].peers[j].peer_id,
                        net.peers[i].peers[j].certainty);
            } else {
                mprintf("[CERT] 0x%04X->0x%04X certainty=%u dist=%.1f ticks\n",
                        net.peers[i].id,
                        net.peers[i].peers[j].peer_id,
                        net.peers[i].peers[j].certainty,
                        dist_scale_to_ticks(net.peers[i].peers[j].distance_scaled));
            }
        }
    }
}

void network_print_positions(void)
{
    mprintf("[POS] 0x%04X %.2f %.2f %.2f\n",
                net.self.id,
                net.self.pos[0],
                net.self.pos[1],
                net.self.pos[2]);
                
    for (int i = 0; i < (int)net.count; i++) {
        mprintf("[POS] 0x%04X %.2f %.2f %.2f\n",
                net.peers[i].id,
                net.peers[i].pos[0],
                net.peers[i].pos[1],
                net.peers[i].pos[2]);
    }
}
