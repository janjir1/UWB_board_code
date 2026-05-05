#ifndef UWB_IMU_EKF_H
#define UWB_IMU_EKF_H

#include <math.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

/* ============================================================================
 * Two-Phase Cooperative UWB + IMU Position Estimator
 * ============================================================================
 *
 * PHASE 1 — ANCHOR SURVEY  (EKF_PHASE_ANCHOR_SURVEY)
 * ─────────────────────────────────────────────────────────────────────────────
 *  Goal   : Determine the 3-D positions of all EKF_NUM_ANCHORS anchors.
 *
 *  Inputs : Inter-anchor UWB range measurements only.
 *           IMU data is IGNORED — anchors are stationary.
 *
 *  Coordinate frame:
 *    EKF_ANCHOR_IDS[0] is pinned at the world origin  (0, 0, 0).
 *    EKF_ANCHOR_IDS[1..3] start with EKF_INIT_P_POS variance and
 *    converge via sequential EKF updates.
 *
 *  Slot table (Phase 1):
 *    Slots hold only non-origin anchors.
 *    If this device is anchor 0         → no self_slot, self is pinned.
 *    If this device is anchor 1/2/3     → self_slot holds self estimate.
 *    If this device is a tag            → no self_slot (tag not estimated).
 *    Non-anchor (tag) peers are ignored entirely.
 *
 *  Convergence condition:
 *    1.  All four anchor IDs are present in the network (not "away").
 *    2.  Every estimated anchor axis satisfies  P[k][k] < EKF_ANCHOR_P_CONVERGED.
 *    3.  At least EKF_ANCHOR_MIN_STEPS have been executed.
 *
 *  On convergence:
 *    Anchor positions are frozen into anchor_pos[EKF_NUM_ANCHORS][3].
 *    LED_R is toggled once:  HAL_GPIO_TogglePin(LED_R_GPIO_Port, LED_R_Pin).
 *    Filter transitions to Phase 2 and is re-initialised for tag tracking.
 *
 *
 * PHASE 2 — TAG LOCALIZATION  (EKF_PHASE_TAG_LOCALIZE)
 * ─────────────────────────────────────────────────────────────────────────────
 *  Goal   : Estimate the 3-D positions of all non-anchor (tag) devices.
 *
 *  Inputs : UWB ranges  — anchor→tag  AND  tag→tag.
 *           IMU         — horizontal speed + vertical speed per tag.
 *
 *  Anchor positions are known and fixed (from Phase 1 result).
 *  Anchors are NOT in the state vector; they are fixed reference points.
 *
 *  Slot table (Phase 2):
 *    Slots hold only non-anchor tag devices.
 *    If this device is a tag            → self_slot allocated.
 *    If this device is an anchor        → no self_slot;
 *                                         self position published from anchor_pos[].
 *    New tags can join at any time      → slot allocated on first appearance.
 *
 *  Disconnect / reconnect:
 *    When a tag disappears:  peer_away[p] = true, P reset (inflated).
 *                            Last known position in x[] is PRESERVED and can
 *                            still be returned by ekf_get_peer_pos().
 *                            During write-back the EKF also writes this last
 *                            known position if the network table still has a
 *                            row for that peer ID.
 *    When it returns:        peer_away cleared, P inflated again so fresh
 *                            measurements quickly reconverge.
 *                            peer_seeded flag and x[] position unchanged
 *                            — this is the "same position as before" guarantee.
 *
 * Joint state vector  x[EKF_MAX_STATE]:
 *   [ x_T0, y_T0, z_T0,   x_T1, y_T1, z_T1,  … ]   (one 3-D block per tag slot)
 *
 * Typical call cadence:
 *   ekf_init();                          // once at startup
 *   // every ~200 ms:
 *   ekf_step(az_self_ms, ah_self_ms);
 *   if (ekf_anchor_ready()) { … }        // Phase 2 active
 * ============================================================================ */


/* ============================================================================
 * Anchor table
 * ============================================================================ */

/** Number of anchors.  One is always the world origin.
 *  Change only when also updating EKF_ANCHOR_IDS[] in ekf.c.            */
#define EKF_NUM_ANCHORS   4U

/** Network IDs of the four anchors.
 *  Defined (not merely declared) in ekf.c — edit them there.
 *  Index 0 = origin anchor, always at (0, 0, 0).                        */
extern const uint16_t EKF_ANCHOR_IDS[EKF_NUM_ANCHORS];


/* ============================================================================
 * Phase 1 convergence thresholds
 * ============================================================================ */

/** Position variance threshold (m²) per axis.  Phase 1 ends when every
 *  estimated anchor axis satisfies  P[k][k] < EKF_ANCHOR_P_CONVERGED.
 *  0.5 m²  ≈  ±0.71 m std-dev.  Increase for a faster but noisier survey;
 *  decrease for a more accurate anchor map (takes longer).               */
#define EKF_ANCHOR_P_CONVERGED   0.2f

/** Minimum EKF steps before convergence is declared, regardless of P.
 *  Guards against premature lock-in during the first noisy measurements. */
#define EKF_ANCHOR_MIN_STEPS     100U


/* ============================================================================
 * Dimensions
 * ============================================================================ */

/** Maximum total EKF slots.
 *  Phase 1 uses at most three slots: anchors 1..3, including self if this
 *  device is one of them.
 *  Phase 2 uses one slot per non-anchor tag, including self when self is a tag.
 *  Keep at 6 unless EKF_MAX_STATE/RAM budget is deliberately reviewed.     */
#define EKF_MAX_PEERS   6
#define EKF_MAX_STATE   (EKF_MAX_PEERS * 3)   /* 18 elements: [x,y,z] × 6 */


/* ============================================================================
 * Filter tuning
 * ============================================================================ */

#define EKF_SIGMA_MIN            0.05f   /* sigma_d @ certainty=255  (m)        */
#define EKF_SIGMA_MAX            0.8f    /* sigma_d @ certainty=0    (m)        */
#define EKF_Q_Z_POS              0.01f  /* Process noise — Z axis   (m²/step)  */
#define EKF_Q_H_FLOOR            0.002f  /* Horizontal noise floor   (m²/step)  */
#define EKF_Q_H_VEL_SCALE        2.0f    /* extra variance per (v·dt)² when moving */
#define EKF_OUTLIER_GATE_POS     2.5f    /* Reject |innov| > gate·√S            */
#define EKF_OUTLIER_GATE_NEG     4.0f
#define EKF_INIT_P_POS           9.0f    /* Initial position variance  (m²)     */
#define EKF_P_MAX                100.0f  /* P diagonal clamp           (m²)     */
#define EKF_DT_NOM_S             0.2f    /* Nominal step period         (s)     */
#define EKF_DT_MAX_S             0.5f    /* dt clamp — absorbs missed steps (s) */

/* IMU / motion tuning — override via project config or ekf.h if desired  */
#define EKF_IMU_DEADBAND_MS      0.15f    /* Speed below this → treated as 0     */
#define EKF_STATIONARY_STEPS     3U      /* Consecutive deadband steps → static */
#define EKF_Q_H_FLOOR_STATIONARY 1e-8f   /* Horiz. noise when stationary (m²)   */
#define EKF_VEL_EMA_ALPHA        0.6f    /* EMA smoothing: 0=frozen  1=raw IMU  */
#define EKF_HDG_MIN_DELTA_M      0.05f   /* Min Δpos to update heading  (m)     */
#define EKF_MAX_STEP_M           0.4f   /* Max predict displacement/step (m)   */
#define EKF_MAX_UPDATE_M         0.5f    /* Max update displacement/step (m)    */


/* ============================================================================
 * Phase enum
 * ============================================================================ */

typedef enum {
    EKF_PHASE_ANCHOR_SURVEY = 0,   /**< Phase 1: estimating anchor positions  */
    EKF_PHASE_TAG_LOCALIZE  = 1,   /**< Phase 2: estimating tag positions     */
} ekf_phase_t;


/* ============================================================================
 * Joint filter state (single static instance inside ekf.c)
 * ============================================================================ */

typedef struct {

    /* ── Fields used by outside code — do NOT remove or reorder ───────────── */
    float    x[EKF_MAX_STATE];                 /**< Joint state vector          */
    float    P[EKF_MAX_STATE][EKF_MAX_STATE];  /**< Joint covariance matrix     */
    float    Q[EKF_MAX_STATE][EKF_MAX_STATE];  /**< Process noise (diagonal)    */
    uint8_t  n_peers;                          /**< Active slots (incl. away)   */
    uint16_t peer_ids[EKF_MAX_PEERS];          /**< Network ID per slot         */
    bool     initialised;                      /**< True after first ekf_step() */

    /* ── Appended fields — safe to add ─────────────────────────────────────── */
    bool     peer_seeded[EKF_MAX_PEERS];       /**< First-fix seed done         */
    bool     peer_away[EKF_MAX_PEERS];         /**< Peer absent from network    */
    bool     peer_imu_valid[EKF_MAX_PEERS];    /**< ≥1 valid IMU packet seen    */
    uint32_t last_tick_ms;                     /**< Timestamp of last step      */
    uint16_t stationary_count[EKF_MAX_PEERS];  /**< Steps below IMU deadband    */
    int8_t   self_slot;                        /**< Slot index for self; -1=none*/

} coop_ekf_t;


/* ============================================================================
 * Public API
 * ============================================================================ */

/**
 * ekf_init
 *   Call once at startup (or after hard reset).
 *   Begins Phase 1: anchor survey.
 *
 *   If called again while Phase 2 is already running, it is treated as a
 *   network self-reconnect event: frozen anchors and last-known tag positions
 *   are preserved, motion priors are cleared, and covariance is inflated.
 *
 *   All four anchor IDs must be set in EKF_ANCHOR_IDS[] (ekf.c) first.
 */
void ekf_init(void);

/**
 * ekf_step
 *   Call every ~200 ms when new ranging data is available.
 *
 *   Phase 1:  az_self_ms / ah_self_ms are ignored (anchors stationary).
 *   Phase 2:  used as self IMU velocity inputs (m/s, gravity removed).
 *
 *   @param az_self_ms  Self vertical velocity   (m/s)
 *   @param ah_self_ms  Self horizontal speed magnitude (m/s, ≥ 0)
 */
void ekf_step(float az_self_ms, float ah_self_ms);

/**
 * ekf_anchor_ready
 *   Returns true once Phase 1 has converged and Phase 2 is active.
 *   The LED was toggled at the exact moment of transition.
 */
bool ekf_anchor_ready(void);

/**
 * ekf_get_phase
 *   Returns the current phase: EKF_PHASE_ANCHOR_SURVEY or EKF_PHASE_TAG_LOCALIZE.
 */
ekf_phase_t ekf_get_phase(void);

/**
 * ekf_get_anchor_positions
 *   Returns a pointer to the frozen anchor position table [EKF_NUM_ANCHORS][3].
 *   Index 0 is always (0,0,0).  Indices 1–3 are valid only after
 *   ekf_anchor_ready() returns true.
 */
const float (*ekf_get_anchor_positions(void))[3];

/**
 * ekf_get_peer_pos
 *   Copy the last-known estimated position of peer_id into pos_out[3].
 *
 *   Phase 1:  works for estimated anchor peers.
 *   Phase 2:  works for tags (estimated) AND anchors (frozen from Phase 1).
 *             Away peers retain their last position — never returns stale zeros.
 *
 *   @return true  if peer_id is known and pos_out was written.
 *           false if peer_id is unknown.
 */
bool ekf_get_peer_pos(uint16_t peer_id, float pos_out[3]);

/**
 * ekf_certainty_to_sigma
 *   Convert certainty byte [0..255] → measurement std-dev in metres.
 *   Exposed for external logging / testing.
 */
float ekf_certainty_to_sigma(uint8_t certainty);

/**
 * ekf_get_state
 *   Read-only pointer to the internal joint filter state.
 *   Use for debug logging only — do NOT modify the returned struct.
 */
const coop_ekf_t *ekf_get_state(void);


#endif /* UWB_IMU_EKF_H */
