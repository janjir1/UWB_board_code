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

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Maximum number of peers (excluding self) the network can track. */
#define NETWORK_MAX_PEERS 7

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
    uint64_t ts;        /**< UWB hardware timestamp (~15 ps resolution). */
    int16_t  rssi_q8;   /**< Total received power, Q8 fixed-point (dBm × 256). */
    int16_t  fp_q8;     /**< First-path power, Q8 fixed-point (dBm × 256). */
} uwb_rx_meas_t;

/**
 * @brief Timestamps captured during an active DS-TWR exchange.
 *
 * Role determines which fields are populated:
 *
 * | Role       | Valid fields                  |
 * |------------|-------------------------------|
 * | Initiator  | poll_tx, resp_rx, final_tx    |
 * | Responder  | poll_rx, resp_tx, final_rx    |
 *
 * Unused fields are zero-initialised and must not be read.
 */
typedef struct {
    uint64_t      poll_tx;  /**< Initiator: time POLL was transmitted. */
    uwb_rx_meas_t poll_rx;  /**< Responder: POLL reception measurement. */

    uint64_t      resp_tx;  /**< Responder: time RESPONSE was transmitted. */
    uwb_rx_meas_t resp_rx;  /**< Initiator: RESPONSE reception measurement. */

    uint64_t      final_tx; /**< Initiator: scheduled FINAL TX time (predicted). */
    uwb_rx_meas_t final_rx; /**< Responder: FINAL reception measurement. */
} twr_timestamps_t;

/**
 * @brief One passive node's observation of the three active DS-TWR frames,
 *        with full signal quality metrics.
 *
 * Captured by listening to POLL, RESPONSE, and FINAL without transmitting.
 * All three fields are always populated by a passive observer.
 *
 * @see twr_observation_simple_t for a timestamp-only variant.
 */
typedef struct {
    uwb_rx_meas_t poll_rx;  /**< Reception measurement of the POLL frame. */
    uwb_rx_meas_t resp_rx;  /**< Reception measurement of the RESPONSE frame. */
    uwb_rx_meas_t final_rx; /**< Reception measurement of the FINAL frame. */
} twr_observation_t;

/**
 * @brief One passive node's observation of the three active DS-TWR frames,
 *        timestamps only (no signal quality metrics).
 *
 * Used when a passive node encodes its observations into its own PASSIVE
 * report frame, where space is limited. All three fields are always populated.
 *
 * @see twr_observation_t for the full-metrics variant.
 */
typedef struct {
    uint64_t poll_rx;   /**< RX timestamp of the POLL frame. */
    uint64_t resp_rx;   /**< RX timestamp of the RESPONSE frame. */
    uint64_t final_rx;  /**< RX timestamp of the FINAL frame. */
} twr_observation_simple_t;

/**
 * @brief One passive node's reception records of other passive report frames.
 *
 * After FINAL, each passive node broadcasts its own MSG_TYPE_PASSIVE frame
 * in ascending order.  Passive node K therefore receives reports from nodes
 * 1 … K−1 before it transmits:
 *
 * @code
 * POLL → RESPONSE → FINAL → PASSIVE_1 → PASSIVE_2 → … → PASSIVE_N
 * @endcode
 *
 * @c passive_rx[i] holds the RX timestamp of the i-th passive report received.
 * Entries beyond what the node received before its own TX are zero.
 * The TWR pair (initiator + responder) never send passive frames, hence -2.
 */
typedef struct {
    uint64_t passive_rx[NETWORK_MAX_PEERS - 2]; /**< RX timestamps of preceding passive reports. */
} passive_observation_t;

/**
 * @brief Single-sided TWR data pair for one passive node.
 *
 * Stores the passive node's own TX timestamp (embedded in its PASSIVE
 * frame) alongside the master's RX measurement of that same frame.
 * Together these form a single-sided TWR pair that the master can use
 * to estimate range to the passive node.
 */
typedef struct {
    uint64_t      passive_tx; /**< TX timestamp reported by the passive node. */
    uwb_rx_meas_t passive_rx; /**< Master's RX measurement of the passive node's frame. */
} ss_twr_t;

/* -----------------------------------------------------------------------
 * Exchange measurement block
 * ----------------------------------------------------------------------- */

/**
 * @brief All measurements captured for the current DS-TWR exchange.
 *
 * Lives flat under @ref network_t — only one exchange is active at a time.
 * Reset with @c memset at the start of each new exchange via
 * @ref network_reset_measurements.
 *
 * Each device fills a different subset depending on its role:
 *
 * | Role        | Fields written                                          |
 * |-------------|---------------------------------------------------------|
 * | Initiator   | twr (poll_tx, resp_rx, final_tx)                        |
 * | Responder   | twr (poll_rx, resp_tx, final_rx)                        |
 * | Passive     | self_twr_observation, self_passive_observation           |
 *
 * The master (initiator) additionally collects passive node reports:
 * - @c twr_observations[i]      — passive node i's view of POLL/RESPONSE/FINAL
 * - @c passive_observations[i]  — passive node i's view of earlier passive frames
 *
 * Valid index range: 0 … @c passive_count - 1.
 */
typedef struct {

    /* ---- Active role (initiator or responder) ---- */
    twr_timestamps_t twr;               /**< Active-role POLL/RESPONSE/FINAL timestamps. */

    /* ---- Passive role (this device only) ---- */
    twr_observation_t    self_twr_observation;      /**< Own view of POLL, RESPONSE, FINAL. */
    passive_observation_t self_passive_observation; /**< Own view of other passive reports
                                                     *   received before self transmitted. */
    uint8_t self_passive_count;                     /**< Number of valid entries in
                                                     *   self_passive_observation.passive_rx[]. */

    /* ---- Master-collected passive data ---- */
    uint16_t              passive_device_id[NETWORK_MAX_PEERS - 2];   /**< Network IDs of passive nodes, in arrival order. */
    ss_twr_t              ss_twr[NETWORK_MAX_PEERS - 2];              /**< SS-TWR pair for each passive node. */
    twr_observation_simple_t twr_observations[NETWORK_MAX_PEERS - 2]; /**< Each passive node's timestamp-only POLL/RESP/FINAL view. */
    passive_observation_t passive_observations[NETWORK_MAX_PEERS - 2];/**< Each passive node's inter-passive RX timestamps. */
    uint8_t passive_count; /**< Number of passive reports received this round.
                            *   Valid indices: 0 … passive_count - 1. */
} measurements_t;

/* -----------------------------------------------------------------------
 * Network peer
 * ----------------------------------------------------------------------- */

/**
 * @brief Identity and position state of one UWB network node.
 *
 * Used for both peers (@c network_t.peers) and self (@c network_t.self).
 * The @c uncertainty field drives DS-TWR target selection: the peer with
 * the highest value is chosen as the next ranging target.  Initialised to
 * 1.0 on first add and updated after each EKF step — typically the trace
 * of the position covariance matrix.
 */
typedef struct {
    uint16_t id;          /**< Network ID. */
    float    pos[3];      /**< Position estimate [x, y, z] in metres.
                           *   Zero-initialised until first fix. */
    float    uncertainty; /**< EKF uncertainty score (0 = unknown, higher = less certain). */
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
    node_t   peers[NETWORK_MAX_PEERS]; /**< Known active peers (excludes self). */
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
void network_set_self_pos(const float pos[3], float uncertainty);

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

/**
 * @brief Reset all measurements for the current exchange.
 *
 * Must be called at the start of each new DS-TWR round before
 * any timestamp setters are called.
 */
void network_reset_measurements(void);

/**
 * @defgroup twr_initiator TWR timestamps — initiator writes
 * @{
 */
/** @brief Store the POLL TX timestamp (initiator). @param ts DW3xxx 40-bit TX timestamp. */
void network_set_twr_poll_tx(uint64_t ts);
/** @brief Store the RESPONSE RX measurement (initiator). @param meas Pointer to the RX measurement. */
void network_set_twr_resp_rx(const uwb_rx_meas_t *meas);
/** @brief Store the FINAL TX timestamp (initiator). @param ts DW3xxx 40-bit TX timestamp. */
void network_set_twr_final_tx(uint64_t ts);
/** @} */

/**
 * @defgroup twr_responder TWR timestamps — responder writes
 * @{
 */
/** @brief Store the POLL RX measurement (responder). @param meas Pointer to the RX measurement. */
void network_set_twr_poll_rx(const uwb_rx_meas_t *meas);
/** @brief Store the RESPONSE TX timestamp (responder). @param ts DW3xxx 40-bit TX timestamp. */
void network_set_twr_resp_tx(uint64_t ts);
/** @brief Store the FINAL RX measurement (responder). @param meas Pointer to the RX measurement. */
void network_set_twr_final_rx(const uwb_rx_meas_t *meas);
/** @} */

/**
 * @brief Get a read-only pointer to the active DS-TWR timestamp set.
 *
 * @return Pointer to @c measurements_t.twr; valid until the next
 *         @ref network_reset_measurements call.
 */
const twr_timestamps_t *network_get_twr(void);

/**
 * @defgroup obs_passive Passive self-observation — own view of active frames
 * @{
 */
/** @brief Store passive node's own RX measurement of POLL.     @param meas Pointer to the RX measurement. */
void network_set_obs_poll_rx(const uwb_rx_meas_t *meas);
/** @brief Store passive node's own RX measurement of RESPONSE. @param meas Pointer to the RX measurement. */
void network_set_obs_resp_rx(const uwb_rx_meas_t *meas);
/** @brief Store passive node's own RX measurement of FINAL.    @param meas Pointer to the RX measurement. */
void network_set_obs_final_rx(const uwb_rx_meas_t *meas);
/**
 * @brief Get a read-only pointer to this device's own TWR observation.
 * @return Pointer to @c measurements_t.self_twr_observation.
 */
const twr_observation_t *network_get_self_twr_observation(void);
/** @} */

/**
 * @defgroup obs_passive_chain Passive self-observation — inter-passive RX timestamps
 * @{
 */
/**
 * @brief Record the RX timestamp of one incoming passive report frame.
 *
 * Call once per received MSG_TYPE_PASSIVE before this device transmits
 * its own passive report.  Index equals arrival order (0-based).
 *
 * @param index  Arrival index (0-based).
 * @param meas   DW3xxx 40-bit RX timestamp.
 * @return @c false if @p index is out of bounds.
 */
bool network_set_passive_report_rx(uint8_t index, const uint64_t meas);
/**
 * @brief Get a read-only pointer to this device's inter-passive observation.
 * @return Pointer to @c measurements_t.self_passive_observation.
 */
const passive_observation_t *network_get_self_passive_observation(void);
/** @brief Get the number of passive reports received before own TX. @return Entry count. */
uint8_t network_get_self_passive_count(void);
/** @} */

/**
 * @defgroup master_passive Master-collected passive data
 * @{
 */
/**
 * @brief Store the master's RX measurement of one passive node's PASSIVE frame.
 *
 * @param index  Passive node index (0-based, order of arrival).
 * @param meas   Master's RX measurement of that PASSIVE frame.
 * @return @c false if @p index is out of bounds.
 */
bool network_set_passive_ss_rx(uint8_t index, const uwb_rx_meas_t *meas);

/**
 * @brief Store the TX timestamp embedded in one passive node's PASSIVE frame.
 *
 * @param index  Passive node index (0-based).
 * @param ts     TX timestamp as reported by the passive node.
 * @return @c false if @p index is out of bounds.
 */
bool network_set_passive_ss_tx(uint8_t index, uint64_t ts);

/**
 * @brief Store one passive node's POLL/RESPONSE/FINAL observation (timestamps only).
 *
 * @param index  Passive node index (0-based).
 * @param obs    Decoded POLL/RESP/FINAL observation from the PASSIVE frame.
 * @return @c false if @p index is out of bounds.
 */
bool network_set_passive_twr_observation(uint8_t index, const twr_observation_simple_t *obs);

/**
 * @brief Store one passive node's inter-passive RX observation.
 *
 * @param index  Passive node index (0-based).
 * @param obs    Decoded inter-passive timestamps from the PASSIVE frame.
 * @return @c false if @p index is out of bounds.
 */
bool network_set_passive_observation(uint8_t index, const passive_observation_t *obs);

/**
 * @brief Increment the master's passive report counter.
 *
 * Call once per fully decoded PASSIVE frame, after all four setters above.
 * No-op if the counter would exceed @c NETWORK_MAX_PEERS - 2.
 */
void network_increment_passive_count(void);

/** @brief Get the number of passive reports collected by the master. @return Report count. */
uint8_t network_get_passive_count(void);

/**
 * @brief Store the network ID of one passive node.
 *
 * @param index  Passive node index (0-based).
 * @param id     Network ID decoded from the PASSIVE frame header.
 * @return @c false if @p index is out of bounds.
 */
bool network_set_passive_device_id(uint8_t index, uint16_t id);

/**
 * @brief Get the network ID of one passive node by index.
 *
 * @param index  Passive node index (0-based).
 * @return Network ID, or 0 if @p index is out of bounds.
 */
uint16_t network_get_passive_device_id(uint8_t index);

/**
 * @brief Get the TX timestamp reported by one passive node.
 *
 * @param index  Passive node index (0-based).
 * @return DW3xxx 40-bit TX timestamp, or 0 if @p index is out of bounds.
 */
uint64_t network_get_passive_ss_tx(uint8_t index);

/**
 * @brief Get one passive node's RX timestamp of POLL.
 * @param index  Passive node index (0-based).
 * @return Timestamp, or 0 if out of bounds.
 */
uint64_t network_get_passive_twr_obs_poll_rx(uint8_t index);

/**
 * @brief Get one passive node's RX timestamp of RESPONSE.
 * @param index  Passive node index (0-based).
 * @return Timestamp, or 0 if out of bounds.
 */
uint64_t network_get_passive_twr_obs_resp_rx(uint8_t index);

/**
 * @brief Get one passive node's RX timestamp of FINAL.
 * @param index  Passive node index (0-based).
 * @return Timestamp, or 0 if out of bounds.
 */
uint64_t network_get_passive_twr_obs_final_rx(uint8_t index);

/**
 * @brief Get the master's RX measurement of one passive node's PASSIVE frame.
 *
 * @param index  Passive node index (0-based).
 * @return @ref uwb_rx_meas_t with all fields zero if @p index is out of bounds.
 */
uwb_rx_meas_t network_get_passive_ss_rx(uint8_t index);
/** @} */

/* -----------------------------------------------------------------------
 * Network API — sequence number
 * ----------------------------------------------------------------------- */

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

#ifdef __cplusplus
}
#endif