/**
 * @file uwb_network.h
 * @brief Network state management for the UWB ranging system.
 *
 * Defines all data types used to represent nodes, timestamps, and
 * measurement results, and exposes the full API for managing the peer
 * list, master election, and per-exchange measurement storage.
 */

#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "messages.h"

#ifdef __cplusplus
extern "C" {
#endif

/* -----------------------------------------------------------------------
 * Positioning and ranging types
 * ----------------------------------------------------------------------- */

/**
 * @brief A single received UWB frame measurement.
 *
 * Bundles the hardware timestamp with the signal quality metrics returned
 * by the DW3xxx driver. TX events use plain @c uint64_t since no signal
 * measurement exists on transmission.
 */
typedef struct {
    uint64_t ts;             /**< UWB hardware timestamp (~15 ps resolution). */
    int16_t  pwr_diff_q8;   /**< Total received power, Q8 fixed-point (dBm × 256). */
} uwb_rx_meas_t;


/* -----------------------------------------------------------------------
 * Exchange measurement block
 * ----------------------------------------------------------------------- */

typedef struct {
    //place to store decoded messeches of this exchange, for master only    
    msg_poll_t poll;
    uwb_rx_meas_t poll_rx;
    bool poll_antenna_unreliable;

    msg_response_t resp;
    uint64_t   resp_tx;

    msg_final_t  final;
    uwb_rx_meas_t final_rx;
    bool final_antenna_unreliable;

    uint8_t passive_count;
    msg_passive_t passive[NETWORK_MAX_PEERS - 2];
    uwb_rx_meas_t passive_rx[NETWORK_MAX_PEERS - 2];
    uint16_t passive_device_id[NETWORK_MAX_PEERS - 2];
    bool passive_antenna_unreliable[NETWORK_MAX_PEERS - 2];

} measurements_t;

/* -----------------------------------------------------------------------
 * Network peer
 * ----------------------------------------------------------------------- */

/**
 * @brief Identity and position state of one UWB network node.
 *
 * Used for both peers (@c network_t.peers) and self (@c network_t.self).
 * The @c certainty field drives DS-TWR target selection: the peer with
 * the lowest certainty is chosen as the next ranging target.
 */

typedef struct {
    uint16_t peer_id;        /**< Network ID of the peer (0 = slot unused). */
    uint16_t distance_scaled;
    uint8_t  certainty;      /**< Certainty score for this peer's distance (0-255). */
} node_peer_state_t;

typedef struct {
    uint16_t id;          /**< Network ID. */
    float    pos[3];      /**< Position estimate [x, y, z] in metres.
                           *   Zero-initialised until first fix. */
    uint8_t  imu_vel_vert;
    uint8_t  imu_vel_horiz;
    node_peer_state_t peers[NETWORK_MAX_PEERS];
} node_t;

/* -----------------------------------------------------------------------
 * Network state
 * ----------------------------------------------------------------------- */

/**
 * @brief Global network state for this device.
 *
 * Managed exclusively through the network API.
 * Direct access to the underlying instance is not exposed.
 */
typedef struct {
    node_t   peers[NETWORK_MAX_PEERS]; 
    uint8_t  expected_seq_num;         /**< Sequence number expected in the next
                                        *   incoming UWB frame. */
    uint8_t  count;                    /**< Number of valid entries in @c peers. */
    uint16_t master_id;                /**< ID of the current master.
                                        *   Equals @c self.id when this device is master. */
    node_t   self;                     /**< This device's own identity and position. */
    bool     acknowledged;             /**< @c true once the master has included
                                        *   @c self.id in a SYNC peer list. */
    measurements_t measurements;       /**< All measurements for the current exchange.
                                        *   Reset at the start of each round. */
} network_t;


node_t *find_peer(uint16_t id);

/* -----------------------------------------------------------------------
 * Network API — initialisation
 * ----------------------------------------------------------------------- */

/**
 * @brief Initialise the network state for this device.
 *
 * Zeroes the entire @ref network_t and sets the own ID.
 * Must be called once before any other network function.
 *
 * @param own_id  This device's 16-bit network ID.
 */
void network_init(uint16_t own_id);

/* -----------------------------------------------------------------------
 * Network API — peer management
 * ----------------------------------------------------------------------- */

/**
 * @brief Add a peer to the network peer list.
 *
 * Does nothing if the peer is already present, if @p id equals own_id,
 * or if the list is full (@ref NETWORK_MAX_PEERS reached).
 *
 * @param id  Network ID of the peer to add.
 */
void network_add_peer(uint16_t id);

/**
 * @brief Remove a peer from the network peer list.
 *
 * Uses swap-with-last to keep the array packed.
 * Does nothing if @p id is not found.
 *
 * @param id  Network ID of the peer to remove.
 */
void network_remove_peer(uint16_t id);

/**
 * @brief Get a read-only pointer to the peer array.
 *
 * @param[out] out_count  Set to the current number of valid peers.
 * @return Pointer to the internal @ref node_t array (valid until next peer-list mutation).
 */
const node_t *network_get_peers(uint8_t *out_count);

/**
 * @brief Get a read-write pointer to the full network state.
 *
 * Prefer the typed accessors where possible; use this only when bulk
 * access to the internal state is unavoidable.
 *
 * @return Pointer to the static @ref network_t instance.
 */
network_t *network_get_network(void);

/**
 * @brief Copy active peer IDs into a caller-supplied array.
 *
 * Skips own_id. Stops when @p max_count entries have been written.
 *
 * @param[out] out_ids    Buffer to receive the peer IDs.
 * @param      max_count  Maximum number of IDs to write.
 * @return Number of IDs written.
 */
uint8_t network_fill_peer_ids(uint16_t *out_ids, uint8_t max_count);

/**
 * @brief Rebuild the peer list from a received SYNC message.
 *
 * Clears the existing peer list and repopulates it from the master's
 * SYNC payload.  The master is inserted as the first entry.
 * Existing position and uncertainty data is preserved for known peers.
 * Peers absent from the SYNC are considered gone.
 *
 * @param master_id   ID of the master that sent the SYNC.
 * @param peer_ids    Array of peer IDs from the SYNC payload.
 * @param peer_count  Number of entries in @p peer_ids.
 */
void network_update_peers_from_sync(uint16_t master_id,
                                    const uint16_t *peer_ids,
                                    uint8_t peer_count);

/**
 * @brief Return the peer index for a given network ID.
 *
 * @param id  Network ID to look up.
 * @return Zero-based index into @c network_t.peers, or -1 if not found.
 */
int8_t network_get_peer_index(uint16_t id);

/* -----------------------------------------------------------------------
 * Network API — master / self
 * ----------------------------------------------------------------------- */

/**
 * @brief Set the current network master.
 *
 * Pass @c own_id to make this device the master.
 *
 * @param id  Network ID of the new master.
 */
void network_set_master(uint16_t id);

/**
 * @brief Get the current master's network ID.
 *
 * @return Master ID, or 0 if no master is set.
 */
uint16_t network_get_master(void);

/**
 * @brief Check whether this device is the network master.
 *
 * @return @c true if @c master_id == @c own_id, @c false otherwise.
 */
bool network_is_master(void);

/**
 * @brief Get this device's own network ID.
 *
 * @return Own ID as set by @ref network_init.
 */
uint16_t network_get_ownid(void);

/**
 * @brief Get the current number of known peers.
 *
 * @return Peer count (0 – @ref NETWORK_MAX_PEERS).
 */
uint8_t network_get_count(void);

/**
 * @brief Set or clear the acknowledged flag.
 *
 * Set to @c true when own_id appears in a master SYNC peer list.
 *
 * @param state  New acknowledged state.
 */
void network_set_acknowledged(bool state);

/**
 * @brief Query whether this device is acknowledged by the master.
 *
 * @return @c true if acknowledged, @c false otherwise.
 */
bool network_is_acknowledged(void);

/* -----------------------------------------------------------------------
 * Network API — position and uncertainty
 * ----------------------------------------------------------------------- */

/**
 * @brief Update this device's own position estimate and uncertainty.
 *
 * @param pos          3-element array [x, y, z] in metres.
 * @param uncertainty  EKF uncertainty score to store.
 */
void network_set_self_pos(const float pos[3]);

/**
 * @brief Return the ID of the peer with the highest position uncertainty.
 *
 * Used by the DS-TWR scheduler to select the next ranging target.
 * Skips own_id.
 *
 * @return Peer ID with highest uncertainty, or 0 if no peers are known.
 */
uint16_t network_get_highest_uncertainty(void);

/* -----------------------------------------------------------------------
 * Network API — measurements
 * ----------------------------------------------------------------------- */


/* -----------------------------------------------------------------------
 * Network API — sequence number
 * ----------------------------------------------------------------------- */

void                  network_reset_measurements(void);
void                  network_store_poll(const msg_poll_t *msg, const uwb_rx_meas_t *rx, const bool antenna_unreliable);
void                  network_store_resp_tx(uint64_t ts);
void                  network_store_final(const msg_final_t *msg, const uwb_rx_meas_t *rx, const bool antenna_unreliable);
bool network_store_passive(uint8_t index, const msg_passive_t *msg,
                           const uwb_rx_meas_t *rx, const uint16_t device_id, const bool antenna_unreliable);
uint8_t               network_get_passive_count(void);

/**
 * @brief Set the expected sequence number for the next incoming frame.
 *
 * @param seq_num  Expected sequence number.
 */
void network_set_expected_seq_num(uint8_t seq_num);

/**
 * @brief Get the expected sequence number for the next incoming frame.
 *
 * @return Current expected sequence number.
 */
uint8_t network_get_expected_seq_num(void);

/* -----------------------------------------------------------------------
 * Per-peer ranging state (distance, k, certainty)
 * ----------------------------------------------------------------------- */



void    network_set_distance   (uint16_t a, uint16_t b, uint16_t dist_scaled);
uint16_t  network_get_distance   (uint16_t a, uint16_t b);
void    network_set_k          (uint16_t a, uint16_t b, double k);
double  network_get_k          (uint16_t a, uint16_t b);
void    network_bump_certainty (uint16_t a, uint16_t b);
void    network_reset_certainty(uint16_t a, uint16_t b);
uint8_t network_get_certainty  (uint16_t a, uint16_t b);
void network_update_certainty(uint16_t a, uint16_t b, uint8_t certainty);
void network_print_certainty(void);

node_peer_state_t *network_get_peer_state(uint16_t owner_id, uint16_t peer_id);
#ifdef __cplusplus
}
#endif