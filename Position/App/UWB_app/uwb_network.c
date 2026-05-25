/**
 * @file uwb_network.c
 * @brief Implementation of the UWB network state management API.
 *
 * The active peer array is a per-round/network-snapshot view. Its indices are
 * intentionally unstable: peers may be removed with swap-with-last packing or
 * rebuilt from SYNC. Code that needs persistent identity must use the 16-bit
 * node ID, never a saved peer index or a saved node_t pointer.
 *
 * Distance state is also treated as a per-round snapshot. The UWB exchange
 * calls network_reset_measurements() at the start of each round, which clears
 * the temporary exchange block and invalidates all stored distances. The
 * distance-calculation stage then writes only the ranges observed in that
 * round. SHARE and EKF run afterwards and therefore see only current-round
 * distances.
 */

#include "cmsis_os.h"
#include "cmsis_os2.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>

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

/**
 * @brief Find an active peer node by its 16-bit ID.
 *
 * @param id  Node ID to search for.
 * @return Pointer to the matching @c node_t, or NULL if not found.
 */
node_t *find_peer(uint16_t id)
{
    for (int i = 0; i < (int)net.count; i++) {
        if (net.peers[i].id == id) {
            return &net.peers[i];
        }
    }
    return NULL;
}

/**
 * @brief Check whether an ID belongs to self or an active peer.
 *
 * @param id  Node ID to check.
 * @return true if the ID is self or found in the active peer list.
 */
static bool network_id_is_active_or_self(uint16_t id)
{
    if (id == net.self.id) {
        return true;
    }
    return (find_peer(id) != NULL);
}

/**
 * @brief Resolve an owner ID to its @c node_t pointer.
 *
 * Returns a pointer to @c net.self when @p owner_id matches own ID,
 * otherwise searches the active peer list.
 *
 * @param owner_id  ID of the node to resolve.
 * @return Pointer to the owner's @c node_t, or NULL if not found.
 */
static node_t *network_find_owner(uint16_t owner_id)
{
    if (owner_id == net.self.id) {
        return &net.self;
    }
    return find_peer(owner_id);
}

/**
 * @brief Find an existing peer slot inside a node's peer table.
 *
 * @param owner    Node whose peer table is searched.
 * @param peer_id  ID of the peer slot to locate.
 * @return Pointer to the matching @c node_peer_state_t, or NULL if not found.
 */
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

/**
 * @brief Find or allocate a peer slot inside a node's peer table.
 *
 * Returns an existing slot if one already exists for @p peer_id, otherwise
 * allocates the first free slot and initialises it. Logs an error if the
 * table is full.
 *
 * @param owner    Node whose peer table is modified.
 * @param peer_id  ID of the peer to find or insert.
 * @return Pointer to the slot, or NULL if the table is full or inputs are invalid.
 */
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
            owner->peers[i].peer_id        = peer_id;
            owner->peers[i].distance_scaled = NETWORK_DISTANCE_INVALID;
            owner->peers[i].certainty      = 0U;
            return &owner->peers[i];
        }
    }

    mprintf("[ERR] network: no free peer slot in node %u for %u\n",
            owner->id, peer_id);
    return NULL;
}

/**
 * @brief Invalidate all stored distances for a single node.
 *
 * Sets @c distance_scaled to @c NETWORK_DISTANCE_INVALID for every
 * occupied peer slot in @p node.
 *
 * @param node  Node whose distances are invalidated. No-op if NULL.
 */
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

/**
 * @brief Invalidate all stored distances across the entire network state.
 *
 * Clears distances in self and every active peer. Called at the start of
 * each ranging round via @ref network_reset_measurements.
 */
static void invalidate_all_distances(void)
{
    invalidate_node_distances(&net.self);

    for (int i = 0; i < (int)net.count; i++) {
        invalidate_node_distances(&net.peers[i]);
    }
}

/**
 * @brief Invalidate the distance to a specific peer ID within one node.
 *
 * @param node        Node whose peer table is updated. No-op if NULL.
 * @param removed_id  Peer ID whose slot is cleared.
 */
static void invalidate_measurements_in_node_for_id(node_t *node, uint16_t removed_id)
{
    if (node == NULL) {
        return;
    }

    for (uint8_t i = 0U; i < NETWORK_MAX_PEERS; i++) {
        if (node->peers[i].peer_id == removed_id) {
            node->peers[i].distance_scaled = NETWORK_DISTANCE_INVALID;
            node->peers[i].certainty       = 0U;
            break;
        }
    }
}

/**
 * @brief Invalidate all distance measurements that reference a removed peer.
 *
 * Walks self and every active peer, clearing any slot whose @c peer_id
 * matches @p removed_id.
 *
 * @param removed_id  ID of the peer that was removed from the network.
 */
static void invalidate_measurements_for_removed_id(uint16_t removed_id)
{
    invalidate_measurements_in_node_for_id(&net.self, removed_id);

    for (int i = 0; i < (int)net.count; i++) {
        invalidate_measurements_in_node_for_id(&net.peers[i], removed_id);
    }
}

/**
 * @brief Check whether an ID already appears in a candidate peer array.
 *
 * @param id         ID to search for.
 * @param new_peers  Candidate peer array.
 * @param new_count  Number of valid entries in @p new_peers.
 * @return true if the ID is present.
 */
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

/**
 * @brief Initialise the network state for this device.
 *
 * Clears all peer and measurement data and sets the local node ID.
 *
 * @param own_id  16-bit short address of this device.
 */
void network_init(uint16_t own_id)
{
    memset(&net, 0, sizeof(net));
    net.self.id     = own_id;
    net.acknowledged = false;
}

/* -------------------------------------------------------------------------
 * Peer management
 * ------------------------------------------------------------------------- */

/**
 * @brief Add a peer to the active peer list.
 *
 * No-op if the ID is zero, already present, or the peer table is full.
 *
 * @param id  16-bit node ID to add.
 */
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

/**
 * @brief Remove a peer from the active peer list.
 *
 * Uses swap-with-last packing to keep the array contiguous. Any previously
 * saved peer index or @c node_t pointer becomes invalid after this call.
 * Also invalidates all distance measurements that reference the removed peer.
 *
 * @param id  16-bit node ID to remove.
 */
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

/**
 * @brief Get a read-only pointer to the active peer array.
 *
 * @param[out] out_count  Filled with the current peer count. May be NULL.
 * @return Pointer to the internal peer array.
 */
const node_t *network_get_peers(uint8_t *out_count)
{
    if (out_count != NULL) {
        *out_count = net.count;
    }
    return net.peers;
}

/**
 * @brief Get a mutable pointer to the full network state.
 *
 * @return Pointer to the internal @c network_t struct.
 */
network_t *network_get_network(void)
{
    return &net;
}

/**
 * @brief Copy active peer IDs into a caller-supplied array.
 *
 * Skips any entry whose ID matches self. Stops when @p max_count is reached.
 *
 * @param[out] out_ids    Destination array.
 * @param      max_count  Maximum number of IDs to write.
 * @return Number of IDs written.
 */
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

/**
 * @brief Rebuild the active peer list from a received SYNC message.
 *
 * Places @p master_id first, then appends the remaining @p peer_ids in order.
 * Peers no longer present in the SYNC list have their measurements invalidated.
 * All distances are cleared at the end to begin a fresh ranging round.
 *
 * @param master_id   Node ID of the SYNC sender (placed first in the list).
 * @param peer_ids    Array of additional peer IDs advertised in the SYNC.
 * @param peer_count  Number of entries in @p peer_ids.
 */
void network_update_peers_from_sync(uint16_t master_id,
                                    const uint16_t *peer_ids,
                                    uint8_t peer_count)
{
    node_t  new_peers[NETWORK_MAX_PEERS];
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

    /* SYNC starts a new active-peer snapshot. Distance calculation will
     * repopulate only the ranges observed in the current round. */
    invalidate_all_distances();
}

/* -------------------------------------------------------------------------
 * Peer index lookup
 * ------------------------------------------------------------------------- */

/**
 * @brief Look up the array index of a peer by ID.
 *
 * @param id  Node ID to find.
 * @return Zero-based index into the peer array, or -1 if not found.
 */
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

/**
 * @brief Set the current master node ID.
 *
 * If the master ID equals own ID, self is removed from the peer list to
 * prevent self-ranging.
 *
 * @param id  16-bit node ID of the master.
 */
void network_set_master(uint16_t id)
{
    net.master_id = id;

    if (id == net.self.id) {
        network_remove_peer(net.self.id);
    }
}

/**
 * @brief Get the current master node ID.
 *
 * @return 16-bit node ID of the master.
 */
uint16_t network_get_master(void)
{
    return net.master_id;
}

/**
 * @brief Check whether this device is currently the master.
 *
 * @return true if own ID matches the stored master ID.
 */
bool network_is_master(void)
{
    return (net.master_id == net.self.id);
}

/**
 * @brief Get this device's own 16-bit node ID.
 *
 * @return Own node ID.
 */
uint16_t network_get_ownid(void)
{
    return net.self.id;
}

/**
 * @brief Get the current active peer count.
 *
 * @return Number of peers in the active peer list.
 */
uint8_t network_get_count(void)
{
    return net.count;
}

/**
 * @brief Set the network acknowledgement flag.
 *
 * @param s  New acknowledgement state.
 */
void network_set_acknowledged(bool s)
{
    net.acknowledged = s;
}

/**
 * @brief Check whether the network has been acknowledged.
 *
 * @return Current value of the acknowledgement flag.
 */
bool network_is_acknowledged(void)
{
    return net.acknowledged;
}

/* -------------------------------------------------------------------------
 * Position and uncertainty
 * ------------------------------------------------------------------------- */

/**
 * @brief Update the stored 3D position of this device.
 *
 * @param pos  Array of three floats [x, y, z]. No-op if NULL.
 */
void network_set_self_pos(const float pos[3])
{
    if (pos == NULL) {
        return;
    }

    net.self.pos[0] = pos[0];
    net.self.pos[1] = pos[1];
    net.self.pos[2] = pos[2];
}

/**
 * @brief Select the peer with the highest ranging uncertainty.
 *
 * Node @c 0xA262 is treated as a priority target: any node other than
 * @c 0xA262 will return it immediately when it is present in the peer list.
 * For @c 0xA262 itself, or when the priority node is absent, the peer with
 * the lowest @c certainty counter is returned. Ties are broken randomly.
 * A peer with no certainty record is returned immediately (certainty unknown).
 *
 * @return ID of the selected peer, or 0 if no peers are available.
 */
uint16_t network_get_highest_uncertainty(void)
{
    const uint16_t PRIORITY_NODE = 0xA262U;

    uint16_t target_id  = 0U;
    uint16_t lowest     = 256U;
    uint8_t  tie_count  = 0U;

    bool check_priority = (network_get_ownid() != PRIORITY_NODE);

    for (int i = 0; i < (int)net.count; i++) {
        uint16_t pid   = net.peers[i].id;
        bool     found = false;

        if (pid == net.self.id) {
            continue;
        }

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

/**
 * @brief Reset the per-round measurement block and invalidate all distances.
 *
 * Called at the start of each ranging round. Certainty counters are preserved
 * for target scheduling; only distances are cleared.
 */
void network_reset_measurements(void)
{
    memset(&net.measurements, 0, sizeof(measurements_t));
    invalidate_all_distances();
}

/**
 * @brief Store an incoming POLL message and its RX metadata.
 *
 * @param msg                 Decoded POLL message. No-op if NULL.
 * @param rx                  RX measurement metadata. No-op if NULL.
 * @param antenna_unreliable  true if the receive antenna quality check failed.
 */
void network_store_poll(const msg_poll_t    *msg,
                        const uwb_rx_meas_t *rx,
                        const bool           antenna_unreliable)
{
    if ((msg == NULL) || (rx == NULL)) {
        return;
    }

    net.measurements.poll                  = *msg;
    net.measurements.poll_rx               = *rx;
    net.measurements.poll_antenna_unreliable = antenna_unreliable;
}

/**
 * @brief Store the RESPONSE TX timestamp.
 *
 * @param ts  DWM3000 TX timestamp of the RESPONSE frame.
 */
void network_store_resp_tx(uint64_t ts)
{
    net.measurements.resp_tx = ts;
}

/**
 * @brief Store an incoming FINAL message and its RX metadata.
 *
 * @param msg                 Decoded FINAL message. No-op if NULL.
 * @param rx                  RX measurement metadata. No-op if NULL.
 * @param antenna_unreliable  true if the receive antenna quality check failed.
 */
void network_store_final(const msg_final_t   *msg,
                         const uwb_rx_meas_t *rx,
                         const bool           antenna_unreliable)
{
    if ((msg == NULL) || (rx == NULL)) {
        return;
    }

    net.measurements.final                  = *msg;
    net.measurements.final_rx               = *rx;
    net.measurements.final_antenna_unreliable = antenna_unreliable;
}

/**
 * @brief Store an incoming PASSIVE message at a given slot index.
 *
 * @param index               Slot index (0-based, max @c NETWORK_MAX_PEERS - 2).
 * @param msg                 Decoded PASSIVE message. Returns false if NULL.
 * @param rx                  RX measurement metadata. Returns false if NULL.
 * @param device_id           Node ID of the passive sender.
 * @param antenna_unreliable  true if the receive antenna quality check failed.
 * @return true on success, false if inputs are invalid or index is out of range.
 */
bool network_store_passive(uint8_t              index,
                           const msg_passive_t  *msg,
                           const uwb_rx_meas_t  *rx,
                           const uint16_t        device_id,
                           const bool            antenna_unreliable)
{
    if ((msg == NULL) || (rx == NULL)) {
        return false;
    }

    if (index >= (uint8_t)(NETWORK_MAX_PEERS - 2U)) {
        return false;
    }

    net.measurements.passive[index]                   = *msg;
    net.measurements.passive_rx[index]                = *rx;
    net.measurements.passive_count                    = (uint8_t)(index + 1U);
    net.measurements.passive_device_id[index]         = device_id;
    net.measurements.passive_antenna_unreliable[index] = antenna_unreliable;

    return true;
}

/**
 * @brief Get a read-only pointer to the current round's measurement block.
 *
 * @return Pointer to the internal @c measurements_t struct.
 */
const measurements_t *network_get_measurements(void)
{
    return &net.measurements;
}

/**
 * @brief Get the number of PASSIVE messages received this round.
 *
 * @return Count of stored passive entries.
 */
uint8_t network_get_passive_count(void)
{
    return net.measurements.passive_count;
}

/* -------------------------------------------------------------------------
 * Sequence number
 * ------------------------------------------------------------------------- */

/**
 * @brief Set the expected sequence number for the next exchange.
 *
 * @param seq_num  Expected sequence number.
 */
void network_set_expected_seq_num(uint8_t seq_num)
{
    net.expected_seq_num = seq_num;
}

/**
 * @brief Get the expected sequence number for the current exchange.
 *
 * @return Expected sequence number.
 */
uint8_t network_get_expected_seq_num(void)
{
    return net.expected_seq_num;
}

/* -------------------------------------------------------------------------
 * Per-peer ranging state
 * ------------------------------------------------------------------------- */

/**
 * @brief Get the ranging state between two active nodes.
 *
 * Returns NULL if either ID is unknown, the slot does not exist, or the
 * stored distance is invalid (i.e. not measured this round).
 *
 * @param owner_id  ID of the node that owns the measurement.
 * @param peer_id   ID of the peer the measurement is against.
 * @return Pointer to the @c node_peer_state_t slot, or NULL.
 */
node_peer_state_t *network_get_peer_state(uint16_t owner_id, uint16_t peer_id)
{
    node_t            *owner;
    node_peer_state_t *slot;

    if (!network_id_is_active_or_self(owner_id) ||
        !network_id_is_active_or_self(peer_id)) {
        return NULL;
    }

    owner = network_find_owner(owner_id);
    slot  = find_peer_slot(owner, peer_id);

    if (slot == NULL) {
        return NULL;
    }

    if (slot->distance_scaled == NETWORK_DISTANCE_INVALID) {
        return NULL;
    }

    return slot;
}

/**
 * @brief Store a scaled distance measurement between two nodes.
 *
 * Creates the peer slot if it does not yet exist.
 *
 * @param a            ID of the measurement owner.
 * @param b            ID of the peer.
 * @param dist_scaled  Scaled distance value to store.
 */
void network_set_distance(uint16_t a, uint16_t b, uint16_t dist_scaled)
{
    node_t            *owner;
    node_peer_state_t *slot;

    if (!network_id_is_active_or_self(a) || !network_id_is_active_or_self(b)) {
        return;
    }

    owner = network_find_owner(a);
    slot  = find_or_add_peer_slot(owner, b);

    if (slot != NULL) {
        slot->distance_scaled = dist_scaled;
    }
}

/**
 * @brief Retrieve the stored scaled distance between two nodes.
 *
 * @param a  ID of the measurement owner.
 * @param b  ID of the peer.
 * @return Stored scaled distance, or @c NETWORK_DISTANCE_INVALID if unavailable.
 */
uint16_t network_get_distance(uint16_t a, uint16_t b)
{
    node_t            *owner;
    node_peer_state_t *slot;

    if (!network_id_is_active_or_self(a) || !network_id_is_active_or_self(b)) {
        return NETWORK_DISTANCE_INVALID;
    }

    owner = network_find_owner(a);
    slot  = find_peer_slot(owner, b);

    if (slot == NULL) {
        return NETWORK_DISTANCE_INVALID;
    }

    return slot->distance_scaled;
}

/**
 * @brief Set the certainty counter for a node pair to a specific value.
 *
 * Creates the peer slot if it does not yet exist.
 *
 * @param a          ID of the measurement owner.
 * @param b          ID of the peer.
 * @param certainty  New certainty value.
 */
void network_update_certainty(uint16_t a, uint16_t b, uint8_t certainty)
{
    node_t            *owner;
    node_peer_state_t *slot;

    if (!network_id_is_active_or_self(a) || !network_id_is_active_or_self(b)) {
        return;
    }

    owner = network_find_owner(a);
    slot  = find_or_add_peer_slot(owner, b);

    if (slot != NULL) {
        slot->certainty = certainty;
    }
}

/**
 * @brief Increment the certainty counter for a node pair by one.
 *
 * Saturates at 255. Creates the peer slot if it does not yet exist.
 *
 * @param a  ID of the measurement owner.
 * @param b  ID of the peer.
 */
void network_bump_certainty(uint16_t a, uint16_t b)
{
    node_t            *owner;
    node_peer_state_t *slot;

    if (!network_id_is_active_or_self(a) || !network_id_is_active_or_self(b)) {
        return;
    }

    owner = network_find_owner(a);
    slot  = find_or_add_peer_slot(owner, b);

    if ((slot != NULL) && (slot->certainty < 255U)) {
        slot->certainty++;
    }
}

/**
 * @brief Reset the certainty counter for a node pair to zero.
 *
 * @param a  ID of the measurement owner.
 * @param b  ID of the peer.
 */
void network_reset_certainty(uint16_t a, uint16_t b)
{
    node_t            *owner;
    node_peer_state_t *slot;

    if (!network_id_is_active_or_self(a) || !network_id_is_active_or_self(b)) {
        return;
    }

    owner = network_find_owner(a);
    slot  = find_peer_slot(owner, b);

    if (slot != NULL) {
        slot->certainty = 0U;
    }
}

/**
 * @brief Get the certainty counter for a node pair.
 *
 * Returns 0 if the slot does not exist or the distance is invalid.
 *
 * @param a  ID of the measurement owner.
 * @param b  ID of the peer.
 * @return Certainty counter value, or 0 if unavailable.
 */
uint8_t network_get_certainty(uint16_t a, uint16_t b)
{
    node_t            *owner;
    node_peer_state_t *slot;

    if (!network_id_is_active_or_self(a) || !network_id_is_active_or_self(b)) {
        return 0U;
    }

    owner = network_find_owner(a);
    slot  = find_peer_slot(owner, b);

    if ((slot == NULL) || (slot->distance_scaled == NETWORK_DISTANCE_INVALID)) {
        return 0U;
    }

    return slot->certainty;
}

/**
 * @brief Print certainty and distance state for all known node pairs.
 *
 * Outputs one line per peer slot for self and each active peer via @c mprintf.
 */
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

/**
 * @brief Print the current 3D position of self and all active peers.
 *
 * Outputs one `[POS]` line per node via @c mprintf.
 */
void network_print_positions(void)
{
    mprintf("[POS] 0x%04X %.2f %.2f %.2f\n",
            net.self.id,
            net.self.pos[0], net.self.pos[1], net.self.pos[2]);

    for (int i = 0; i < (int)net.count; i++) {
        mprintf("[POS] 0x%04X %.2f %.2f %.2f\n",
                net.peers[i].id,
                net.peers[i].pos[0], net.peers[i].pos[1], net.peers[i].pos[2]);
    }
}