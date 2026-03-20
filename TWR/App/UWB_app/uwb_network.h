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
    int16_t  fp_q8;     /**< First-path power,      Q8 fixed-point (dBm × 256). */
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
    uint64_t      poll_tx;   /**< Initiator: time POLL was transmitted. */
    uwb_rx_meas_t poll_rx;   /**< Responder: POLL reception. */

    uint64_t      resp_tx;   /**< Responder: time RESPONSE was transmitted. */
    uwb_rx_meas_t resp_rx;   /**< Initiator: RESPONSE reception. */

    uint64_t      final_tx;  /**< Initiator: time FINAL was transmitted (predicted). */
    uwb_rx_meas_t final_rx;  /**< Responder: FINAL reception. */
} twr_timestamps_t;

/**
 * @brief One passive node's observation of the three active DS-TWR frames.
 *
 * Captured by listening to POLL, RESPONSE, and FINAL without transmitting.
 * Every passive node always populates all three fields.
 */
typedef struct {
    uwb_rx_meas_t poll_rx;   /**< Reception of the POLL frame. */
    uwb_rx_meas_t resp_rx;   /**< Reception of the RESPONSE frame. */
    uwb_rx_meas_t final_rx;  /**< Reception of the FINAL frame. */
} twr_observation_t;

/**
 * @brief One passive node's reception records of other passive report frames.
 *
 * After FINAL, each passive node broadcasts its own MSG_TYPE_PASSIVE frame
 * in ascending order. Passive node K therefore receives reports from nodes
 * 1 … K-1 before it transmits:
 *
 *   POLL → RESPONSE → FINAL → PASSIVE_1 → PASSIVE_2 → … → PASSIVE_N
 *
 * passive_rx[i] corresponds to the i-th passive report received.
 * Entries beyond what the node received before transmitting are zero.
 * The TWR pair (initiator + responder) never send passive frames, hence -2.
 */
typedef struct {
    uwb_rx_meas_t passive_rx[NETWORK_MAX_PEERS - 2];
} passive_observation_t;

/* -----------------------------------------------------------------------
 * Exchange measurement block
 * ----------------------------------------------------------------------- */

/**
 * @brief All measurements captured for the current DS-TWR exchange.
 *
 * Lives flat under @c network_t — only one exchange is active at a time.
 * Reset with @c memset at the start of each new exchange.
 *
 * Each device fills a different subset depending on its role:
 *
 * | Role        | Fields written                                          |
 * |-------------|--------------------------------------------------------|
 * | Initiator   | twr (poll_tx, resp_rx, final_tx)                       |
 * | Responder   | twr (poll_rx, resp_tx, final_rx)                       |
 * | Passive     | self_twr_observation, self_passive_observation         |
 *
 * Additionally, the master (initiator) collects and stores reports from
 * every passive node after FINAL:
 *   twr_observations[i]     — passive node i's view of POLL/RESPONSE/FINAL
 *   passive_observations[i] — passive node i's view of earlier passive frames
 * Valid range: indices 0 … passive_count-1.
 */
typedef struct {

    /* ---- Active role (initiator or responder) ---- */
    twr_timestamps_t twr;

    /* ---- Passive role (this device only) ---- */
    twr_observation_t     self_twr_observation;      /*< Own view of POLL, RESPONSE, FINAL. */
    passive_observation_t self_passive_observation;  /*< Own view of other passive reports
                                                      *   received before self transmitted. */
    uint8_t               self_passive_count;    /*< Number of valid entries in
                                                  *   self_passive_observation.passive_rx[]. */

    // ---- Master-collected passive data ---- 
    twr_observation_t     twr_observations[NETWORK_MAX_PEERS - 2];      /*< Each passive node's POLL/RESP/FINAL view. */
    passive_observation_t passive_observations[NETWORK_MAX_PEERS - 2];  /*< Each passive node's inter-passive view. */
    uint8_t               passive_count;  /**< Number of passive reports received.
                                           *   Valid indices: 0 … passive_count-1. */

} measurements_t;

/* -----------------------------------------------------------------------
 * Network peer
 * ----------------------------------------------------------------------- */

/**
 * @brief Identity and position state of one UWB network node.
 *
 * Used for both peers (@c network_t.peers) and self (@c network_t.self).
 * The @c uncertainty field drives DS-TWR target selection: the peer with
 * the highest value is chosen as the next ranging target. Initialised to
 * 0 (unknown) and updated after each EKF step — typically the trace of
 * the position covariance matrix.
 */
typedef struct {
    uint16_t id;           /**< Network ID. */
    float    pos[3];       /**< Position estimate [x, y, z] in metres.
                            *   Zero-initialised until first fix. */
    float    uncertainty;  /**< EKF uncertainty score (0 = unknown, higher = less certain). */
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
    uint8_t  count;                    /**< Number of valid entries in @c peers. */
    uint16_t master_id;                /**< ID of the current master.
                                        *   Equals @c self.id when this device is master. */
    node_t   self;                     /**< This device's own identity and position. */
    bool     acknowledged;             /**< True once master has included
                                        *   @c self.id in a SYNC peer list. */

    measurements_t measurements;       /**< All measurements for the current exchange.
                                        *   Reset with memset at the start of each round. */
} network_t;

/* -----------------------------------------------------------------------
 * Network API
 * --------------------------------------------------------------------- */

/**
 * @brief Initialise the network state for this device.
 * @param own_id  This device's 16-bit network ID.
 */
void network_init(uint16_t own_id);

/**
 * @brief Add a peer to the network peer list.
 *
 * If the peer is already present its entry is left unchanged.
 * Does nothing if the list is full (@c NETWORK_MAX_PEERS reached).
 *
 * @param id  Network ID of the peer to add.
 */
void network_add_peer(uint16_t id);

/**
 * @brief Remove a peer from the network peer list.
 *
 * Does nothing if @p id is not found.
 *
 * @param id  Network ID of the peer to remove.
 */
void network_remove_peer(uint16_t id);

/**
 * @brief Get a read-only pointer to the peer array.
 *
 * @param[out] out_count  Set to the current number of peers.
 * @return                Pointer to the internal peer array.
 */
const node_t  *network_get_peers(uint8_t *out_count);

/**
 * @brief Set the current network master.
 *
 * Setting this to @c own_id makes this device the master.
 *
 * @param id  Network ID of the new master.
 */
void network_set_master(uint16_t id);

/**
 * @brief Get the current master's network ID.
 * @return Master ID, or 0 if no master is set.
 */
uint16_t network_get_master(void);

/**
 * @brief Check whether this device is the network master.
 * @return Non-zero if @c master_id == @c own_id, zero otherwise.
 */
bool network_is_master(void);

/**
 * @brief Get this device's own network ID.
 * @return Own ID as set by @c network_init().
 */
uint16_t network_get_ownid(void);

/**
 * @brief Get the current number of known peers.
 * @return Peer count (0 – @c NETWORK_MAX_PEERS).
 */
uint8_t network_get_count(void);

/**
 * @brief Copy active peer IDs into a caller-supplied array.
 *
 * Skips own_id and inactive entries.  Stops when @p max_count is reached.
 *
 * @param[out] out_ids    Buffer to receive the peer IDs.
 * @param      max_count  Maximum number of IDs to write.
 * @return                Number of IDs written.
 */
uint8_t network_fill_peer_ids(uint16_t *out_ids, uint8_t max_count);

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
 * @return @c true if acknowledged, @c false otherwise.
 */
bool network_is_acknowledged(void);

/**
 * @brief Rebuild the peer list from a received SYNC message.
 *
 * Clears the existing peer list and repopulates it from the master's
 * peer list. The master itself is added as the first entry.
 * The master is treated as the single source of truth — peers absent
 * from the SYNC are considered gone.
 *
 * @param master_id   ID of the master that sent the SYNC.
 * @param peer_ids    Array of peer IDs from the SYNC payload.
 * @param peer_count  Number of entries in @p peer_ids.
 */
void network_update_peers_from_sync(uint16_t master_id,
                                    const uint16_t *peer_ids,
                                    uint8_t peer_count);

/** Returns the ID of the device with the highest position uncertainty. */
uint16_t        network_get_highest_uncertainty(void);

/**
 * @brief Reset all measurements for the current exchange.
 *
 * Must be called at the start of each new DS-TWR round before
 * any timestamps are written.
 */
void network_reset_measurements(void);

/* twr_timestamps_t — initiator */
void network_set_twr_poll_tx(uint64_t ts);
void network_set_twr_resp_rx(const uwb_rx_meas_t *meas);
void network_set_twr_final_tx(uint64_t ts);

/* twr_timestamps_t — responder */
void network_set_twr_poll_rx(const uwb_rx_meas_t *meas);
void network_set_twr_resp_tx(uint64_t ts);
void network_set_twr_final_rx(const uwb_rx_meas_t *meas);

/* twr_timestamps_t — read */
const twr_timestamps_t *network_get_twr(void);

/* twr_observation_t — passive own view of active frames */
void network_set_obs_poll_rx(const uwb_rx_meas_t *meas);
void network_set_obs_resp_rx(const uwb_rx_meas_t *meas);
void network_set_obs_final_rx(const uwb_rx_meas_t *meas);
const twr_observation_t *network_get_self_twr_observation(void);

/* passive_observation_t — passive own view of passive reports */
bool    network_set_passive_report_rx(uint8_t index, const uwb_rx_meas_t *meas);
const   passive_observation_t *network_get_self_passive_observation(void);
uint8_t network_get_self_passive_count(void);

network_t *network_get_network(void);

#ifdef __cplusplus
}
#endif