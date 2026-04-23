#ifndef UWB_IMU_EKF_H
#define UWB_IMU_EKF_H

#include <math.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

/* ============================================================================
 * Cooperative UWB + IMU Position Estimator — Joint EKF
 * ============================================================================
 *
 * Architecture:
 *   - Self is always the fixed origin [0, 0, 0].
 *   - ALL peer positions are estimated JOINTLY in one single filter.
 *   - Every inter-node distance (self→peer AND peer→peer) is used as a
 *     measurement, each weighted by its own certainty.
 *
 * Joint state vector x[EKF_MAX_STATE]:
 *   [ x_P0, y_P0, z_P0,
 *     x_P1, y_P1, z_P1,
 *     ...
 *     x_Pn, y_Pn, z_Pn ]
 *
 *   Peers are indexed by their slot in network_t.peers[].
 *   Only positions are tracked — velocity is handled implicitly via Q.
 *
 * Predict step:
 *   Each peer's position is nudged by its decoded IMU velocity minus self's
 *   IMU velocity (relative motion). Horizontal direction unknown — inflates
 *   X/Y process noise. Vertical direction known — low Z noise.
 *
 * Update step:
 *   For every pair (i, j) with a valid distance measurement:
 *     - Self→peer measurements: anchor = [0,0,0], tag = peer state block
 *     - Peer→peer measurements: both endpoints are in the joint state,
 *       Jacobian spans two 3-element blocks simultaneously.
 *   Each update uses certainty-derived sigma_d as measurement noise.
 *
 * Call once per 200ms:
 *   ekf_step(az_self_ms, ah_self_ms);
 *
 * Positions written back to node_t.pos[3] for every peer after each step.
 * ============================================================================ */

/* ---- Dimensions ----------------------------------------------------------- */
#define EKF_MAX_PEERS   6                       /* NETWORK_MAX_PEERS - 1 (self)  */
#define EKF_MAX_STATE   (EKF_MAX_PEERS * 3)     /* 18: [x,y,z] per peer          */

/* ---- Tuning --------------------------------------------------------------- */
#define EKF_SIGMA_MIN       0.05f   /* sigma_d for certainty=255 (best)  (m) */
#define EKF_SIGMA_MAX       2.0f    /* sigma_d for certainty=0   (worst) (m) */
#define EKF_Q_Z_POS         0.002f  /* Process noise — Z position            */
#define EKF_Q_H_FLOOR       0.1f    /* Min horizontal noise floor            */
#define EKF_OUTLIER_GATE    3.0f    /* Reject if |innov| > GATE * sigma_d    */
#define EKF_INIT_P_POS      25.0f   /* Initial position variance (m²) ~±5m  */

/* ---- Joint filter state (single static instance in .c) ------------------- */
typedef struct {
    float   x[EKF_MAX_STATE];               /* Joint state vector             */
    float   P[EKF_MAX_STATE][EKF_MAX_STATE];/* Joint covariance matrix        */
    float   Q[EKF_MAX_STATE][EKF_MAX_STATE];/* Process noise matrix           */
    uint8_t n_peers;                        /* Active peer count              */
    uint16_t peer_ids[EKF_MAX_PEERS];       /* peer_ids[i] -> state block i   */
    bool    initialised;                    /* true after first ekf_step call */
} coop_ekf_t;

/* ============================================================================
 * ekf_init
 *   Call once at startup or whenever the peer list changes significantly.
 *   Resets the joint filter and re-registers all current peers from the
 *   network layer. Large initial P — converges within a few 200ms ticks.
 * ============================================================================ */
void ekf_init(void);

/* ============================================================================
 * ekf_step
 *   Call every 200ms when sensor data arrives.
 *   Internally:
 *     1. Syncs peer list from network (adds/removes peers as needed)
 *     2. Predict: applies relative IMU motion to all peer state blocks
 *     3. Update: iterates all valid inter-node distances from the network,
 *                applies sequential EKF updates weighted by certainty
 *     4. Writes x,y,z back to node_t.pos for every peer
 *
 *   az_self_ms : self vertical velocity this tick (m/s), gravity removed
 *   ah_self_ms : self horizontal speed magnitude this tick (m/s), >= 0
 * ============================================================================ */
void ekf_step(float az_self_ms, float ah_self_ms);

/* ============================================================================
 * ekf_certainty_to_sigma
 *   Convert certainty byte [0..255] -> sigma_d in metres.
 *   Exposed for logging/testing.
 * ============================================================================ */
float ekf_certainty_to_sigma(uint8_t certainty);

/* ============================================================================
 * ekf_get_state
 *   Returns a read-only pointer to the internal joint filter state.
 *   Use for debug logging only — do not modify.
 * ============================================================================ */
const coop_ekf_t *ekf_get_state(void);

#endif /* UWB_IMU_EKF_H */
