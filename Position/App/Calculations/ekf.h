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
 *   Peers are indexed by their EKF slot (assigned at join time, never freed).
 *   Absent peers are marked peer_away[p]=true; their slot is retained so that
 *   the last-known position survives and convergence is preserved on return.
 *   Only positions are tracked — velocity is handled implicitly via Q.
 *
 * Predict step:
 *   Variable dt from HAL_GetTick(), capped at EKF_DT_MAX_S.
 *   Each peer's Z position is nudged by its relative IMU vertical velocity.
 *   Horizontal process noise = RMS(peer_vh, self_vh)^2 * dt^2 + floor.
 *   Vertical process noise = EKF_Q_Z_POS scaled by dt / dt_nom.
 *
 * Update step:
 *   For every pair (i, j) with a valid distance measurement:
 *     Self→peer:   anchor = [0,0,0] (self), tag = peer state block.
 *     Peer→peer:   both endpoints in the joint state; Jacobian spans two
 *                  3-element blocks simultaneously.
 *   Each update uses certainty-derived sigma_d as measurement noise (R).
 *   Outlier gate: |innov| > EKF_OUTLIER_GATE × sqrt(S), where
 *                 S = H*P*H^T + R  (dynamic, not fixed sigma_d).
 *   Covariance update: Joseph form — preserves symmetry and PSD under
 *                      floating-point arithmetic.
 *
 * Call once per ~200 ms:
 *   ekf_step(az_self_ms, ah_self_ms);
 *
 * Positions written back to node_t.pos[3] for every present peer after step.
 * Self position set to [0,0,0] via network_set_self_pos() every step.
 * ============================================================================ */

/* ---- Dimensions ----------------------------------------------------------- */
#define EKF_MAX_PEERS   6                       /* NETWORK_MAX_PEERS - 1 (self)  */
#define EKF_MAX_STATE   (EKF_MAX_PEERS * 3)     /* 18: [x,y,z] per peer          */

/* ---- Tuning --------------------------------------------------------------- */
#define EKF_SIGMA_MIN       0.05f   /* sigma_d for certainty=255 (best)   (m)   */
#define EKF_SIGMA_MAX       2.0f    /* sigma_d for certainty=0   (worst)  (m)   */
#define EKF_Q_Z_POS         0.002f  /* Process noise — Z position variance (m²) */
#define EKF_Q_H_FLOOR       0.005f  /* Min horizontal noise floor (m²/step)     */
                                    /* (was 0.1f — was added as displacement,   */
                                    /*  not variance; now used correctly as m²) */
#define EKF_OUTLIER_GATE    3.0f    /* Reject if |innov| > GATE × sqrt(S)       */
                                    /* (S includes P term, not just sigma_d)    */
#define EKF_INIT_P_POS      25.0f   /* Initial position variance (m²) ~±5 m     */
#define EKF_P_MAX           400.0f  /* P diagonal clamp (m²) ~±20 m             */
#define EKF_DT_NOM_S        0.2f    /* Nominal step duration (s)                */
#define EKF_DT_MAX_S        1.0f    /* dt cap — absorbs missed/late steps (s)   */

/* ---- Joint filter state (single static instance in ekf.c) ---------------- */
typedef struct {

    /* --- Fields used by outside code — do NOT remove or reorder ----------- */
    float    x[EKF_MAX_STATE];                  /* Joint state vector             */
    float    P[EKF_MAX_STATE][EKF_MAX_STATE];   /* Joint covariance matrix        */
    float    Q[EKF_MAX_STATE][EKF_MAX_STATE];   /* Process noise (diagonal only)  */
    uint8_t  n_peers;                           /* Total peer slots (incl. away)  */
    uint16_t peer_ids[EKF_MAX_PEERS];           /* peer_ids[i] → state block i   */
    bool     initialised;                       /* true after first ekf_step call */

    /* --- New fields — appended below existing fields; safe to add --------- */
    bool     peer_seeded[EKF_MAX_PEERS];        /* Per-peer first-fix seed done   */
    bool     peer_away[EKF_MAX_PEERS];          /* Peer absent from network layer */
    bool     peer_imu_valid[EKF_MAX_PEERS];     /* Peer has sent ≥1 valid message;
                                                 * gates IMU use (prevents the
                                                 * zero-init → -4 m/s bug)        */
    uint32_t last_tick_ms;                      /* Timestamp of last ekf_step()   */

} coop_ekf_t;

/* ============================================================================
 * ekf_init
 *   Call once at startup (or after a hard reset).
 *   Registers all current network peers and initialises the joint filter.
 *
 *   Do NOT call again when peers join or leave — that is handled
 *   automatically inside ekf_step() via the internal sync_peers() function.
 * ============================================================================ */
void ekf_init(void);

/* ============================================================================
 * ekf_step
 *   Call every ~200 ms when sensor data is ready.
 *
 *   Internally:
 *     1. Syncs peer list from network layer:
 *          - New peers   → new slot added with large P, zero cross-covariance.
 *          - Absent peers → marked away; covariance reset immediately to
 *                           prevent ghost-correlation updates.
 *          - Returning peers → away flag cleared; P and seeding reset so the
 *                              filter re-acquires from first measurement.
 *     2. Predict: applies relative IMU Z motion; updates Q; P += Q (diagonal);
 *                 clamps P diagonal to EKF_P_MAX.
 *     3. Update: all valid self→peer and peer→peer distances (sequential EKF).
 *     4. Write-back: copies estimated x,y,z to node_t.pos for present peers.
 *     5. Sets self position to [0,0,0] via network_set_self_pos().
 *
 *   az_self_ms : self vertical velocity this tick (m/s), gravity removed
 *   ah_self_ms : self horizontal speed magnitude this tick (m/s), >= 0
 * ============================================================================ */
void ekf_step(float az_self_ms, float ah_self_ms);

/* ============================================================================
 * ekf_certainty_to_sigma
 *   Convert certainty byte [0..255] -> measurement sigma_d in metres.
 *   Exposed for logging/testing.
 * ============================================================================ */
float ekf_certainty_to_sigma(uint8_t certainty);

/* ============================================================================
 * ekf_get_state
 *   Returns a read-only pointer to the internal joint filter state.
 *   Use for debug logging only — do not modify the returned struct.
 * ============================================================================ */
const coop_ekf_t *ekf_get_state(void);

/* ============================================================================
 * ekf_get_peer_pos
 *   Copy the last-known estimated position of a peer into pos_out[3].
 *   Works for both present and away peers (away peers retain their last fix).
 *
 *   Returns true  if a slot exists for peer_id (position written).
 *   Returns false if peer_id is unknown (pos_out unchanged).
 * ============================================================================ */
bool ekf_get_peer_pos(uint16_t peer_id, float pos_out[3]);

#endif /* UWB_IMU_EKF_H */