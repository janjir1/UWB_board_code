#include "ekf.h"
#include "main.h"                       /* LED_R_GPIO_Port, LED_R_Pin          */
#include "../UWB_app/uwb_network.h"
#include "../Calculations/distance.h"
#include <math.h>
#include <string.h>
#include "../Generic/my_print.h"

/* HAL tick — same pattern as before; avoids pulling in the full HAL chain.  */
extern uint32_t HAL_GetTick(void);

/* ============================================================================
 * INITIAL POSITION ESTIMATES
 * ─────────────────────────────────────────────────────────────────────────────
 * Rough starting positions for each known node, in metres relative to
 * Anchor 0 (world origin).  A few metres of error is fine — the EKF will
 * converge to the correct position.  The important thing is that the signs
 * are correct so the filter starts on the right side of the origin.
 *
 * Nodes not listed here get a small fixed jitter (±0.5 m) instead of a
 * random position, which also avoids the degenerate all-zeros start.
 *
 * Format: { node_id, x_m, y_m, z_m }
 * ============================================================================ */
/* Spread of the Xorshift32 random seed used for unknown tag nodes (metres). */
#define EKF_INIT_RAND_M  2.0f

typedef struct { uint16_t id; float x, y, z; } ekf_node_hint_t;

static const ekf_node_hint_t ekf_pos_hints[] = {
    /* ── anchors ── */
    { 0x63D8u,  0.0f,  0.0f,  0.0f },   /* ANCHOR 0 — origin              */
    { 0x91EDu,  1.0f,  1.0f,  2.0f },   /* ANCHOR 1 — ~5 m along X        */
    { 0xC019u,  -2.0f,  4.0f,  -0.20f },   /* ANCHOR 2 — corner              */
    { 0x28CCu,  -3.0f,  4.0f,  2.5f },   /* ANCHOR 3 — elevated centre     */
    /* ── tags / peers — add rows as needed ── */
    /* { 0xAAAAu,  1.0f,  1.0f,  1.0f }, */
};
#define EKF_POS_HINTS_N  (sizeof(ekf_pos_hints) / sizeof(ekf_pos_hints[0]))

/* Fallback jitter when a node has no hint entry (avoids exact zero). */

/* ============================================================================
 * ANCHOR TABLE
 * ─────────────────────────────────────────────────────────────────────────────
 * Edit the four network IDs below to match your hardware.
 * Index 0 is the world-origin anchor — always pinned at (0, 0, 0).
 * Indices 1–3 are surveyed in Phase 1; their positions are unknown at build time.
 *
 * Physical placement recommendation for good geometry (low DOP):
 *   Anchor 0  —  one corner of the room  (origin)
 *   Anchor 1  —  opposite corner or at least >2 m away on any axis
 *   Anchor 2  —  off the line formed by 0–1 (different Y or Z)
 *   Anchor 3  —  elevated or offset so all four are non-coplanar
 * ============================================================================ */
const uint16_t EKF_ANCHOR_IDS[EKF_NUM_ANCHORS] = {
    0x63D8u,   /* ANCHOR 0 — origin (0, 0, 0) */
    0x91EDu,   /* ANCHOR 1                     */
    0xC019u,   /* ANCHOR 2                     */
    0x28CCu,   /* ANCHOR 3                     */
};

/* ============================================================================
 * Internal state
 * ============================================================================ */

static coop_ekf_t  ekf;
static ekf_phase_t ekf_phase;

/* Anchor positions frozen at end of Phase 1.
 * anchor_pos[0] is always (0,0,0); indices 1–3 filled on Phase 1 convergence. */
static float anchor_pos[EKF_NUM_ANCHORS][3];

/* Phase 1 step counter (for EKF_ANCHOR_MIN_STEPS gate).                      */
static uint32_t phase1_steps;

/* Static world-origin constant — used throughout to avoid stack allocation.   */
static const float k_origin[3] = { 0.0f, 0.0f, 0.0f };

/* ── Per-slot motion prior ──────────────────────────────────────────────── */
static float slot_vel_ema_h[EKF_MAX_PEERS];  /* horizontal EMA speed (m/s)  */
static float slot_vel_ema_z[EKF_MAX_PEERS];  /* vertical   EMA speed (m/s)  */
static float slot_last_x   [EKF_MAX_PEERS];
static float slot_last_y   [EKF_MAX_PEERS];
static float slot_hdg_x    [EKF_MAX_PEERS];  /* heading unit-vector x       */
static float slot_hdg_y    [EKF_MAX_PEERS];  /* heading unit-vector y       */
static bool  slot_hdg_valid[EKF_MAX_PEERS];


/* ============================================================================
 * Anchor helpers
 * ============================================================================ */

/** Returns true when id matches one of the hardcoded anchor IDs. */
static bool is_anchor_id(uint16_t id)
{
    for (uint8_t a = 0; a < EKF_NUM_ANCHORS; a++)
        if (EKF_ANCHOR_IDS[a] == id) return true;
    return false;
}

/** Returns anchor index [0..EKF_NUM_ANCHORS-1], or -1 if id is not an anchor. */
static int anchor_index_of(uint16_t id)
{
    for (int a = 0; a < (int)EKF_NUM_ANCHORS; a++)
        if (EKF_ANCHOR_IDS[a] == id) return a;
    return -1;
}

/**
 * anchor_idx_to_slot
 *   Maps anchor index to its EKF slot index in the current Phase 1 table.
 *   Anchor 0 (origin) has no slot → returns -1.
 *   Self anchor        → returns ekf.self_slot.
 *   Peer anchor        → searches peer_ids[]; returns -1 if not found yet.
 */
static int anchor_idx_to_slot(int anchor_idx)
{
    if (anchor_idx == 0) return -1;   /* origin is a fixed reference, no slot */

    uint16_t own_id = network_get_ownid();
    if (EKF_ANCHOR_IDS[anchor_idx] == own_id)
        return (int)ekf.self_slot;

    for (int p = 0; p < (int)ekf.n_peers; p++)
        if (ekf.peer_ids[p] == EKF_ANCHOR_IDS[anchor_idx]) return p;

    return -1;
}


/* ============================================================================
 * Low-level utilities  (unchanged from v1)
 * ============================================================================ */

static float range_scaled_to_m(uint16_t dist_scaled)
{
    float ticks;

    if ((dist_scaled == 0xFFFFU) || (dist_scaled == 0x0000U)) {
        return -1.0f;
    }

    /* Keep EKF-side arithmetic in single precision.  dist_scale_to_ticks() may
     * be implemented elsewhere, but the filter does not propagate double math. */
    ticks = (float)dist_scale_to_ticks(dist_scaled);
    if (ticks <= 0.0f) {
        return -1.0f;
    }

    return ticks * (float)METERS_PER_TICK;
}

float ekf_certainty_to_sigma(uint8_t certainty)
{
    float q = (float)certainty / 255.0f;
    return EKF_SIGMA_MIN + (EKF_SIGMA_MAX - EKF_SIGMA_MIN) * (1.0f - q);
}

static int8_t find_ekf_slot(uint16_t peer_id)
{
    for (int8_t p = 0; p < (int8_t)ekf.n_peers; p++) {
        if (ekf.peer_ids[p] == peer_id) {
            return p;
        }
    }
    return -1;
}

static bool network_contains_id(uint16_t id)
{
    network_t *net = network_get_network();

    if (id == network_get_ownid()) {
        return true;
    }

    if (net == NULL) {
        return false;
    }

    for (int k = 0; k < (int)net->count; k++) {
        if (net->peers[k].id == id) {
            return true;
        }
    }

    return false;
}

static node_peer_state_t *get_peer_state_symmetric(uint16_t id_a, uint16_t id_b)
{
    node_peer_state_t *ps = network_get_peer_state(id_a, id_b);

    if (ps == NULL) {
        ps = network_get_peer_state(id_b, id_a);
    }

    return ps;
}

static void publish_self_position(const float pos[3])
{
    float pos_tmp[3];
    network_t *net;
    uint16_t own_id;

    pos_tmp[0] = pos[0];
    pos_tmp[1] = pos[1];
    pos_tmp[2] = pos[2];
    mprintf("[EKF] publish_self pos=(%.3f, %.3f, %.3f)\n",
            (double)pos[0], (double)pos[1], (double)pos[2]);
    network_set_self_pos(pos_tmp);

    /* Compatibility with the existing network snapshot model: some SYNC
     * layouts keep this device's own ID inside net->peers[] as well as in
     * net->self.  Keep that duplicate row coherent for modules that inspect
     * net->peers[] directly.  EKF logic itself still treats self via
     * network_get_ownid() / self_slot and never allocates a duplicate slot. */
    net = network_get_network();
    own_id = network_get_ownid();

    if (net != NULL) {
        for (int k = 0; k < (int)net->count; k++) {
            if (net->peers[k].id == own_id) {
                net->peers[k].pos[0] = pos[0];
                net->peers[k].pos[1] = pos[1];
                net->peers[k].pos[2] = pos[2];
                break;
            }
        }
    }
}


/* ============================================================================
 * Slot management
 * ============================================================================ */

static void reset_slot_covariance(int slot, int n_active)
{
    int base = slot * 3;
    for (int k = 0; k < 3; k++) {
        int row = base + k;
        for (int j = 0; j < n_active; j++) {
            ekf.P[row][j] = 0.0f;
            ekf.P[j][row] = 0.0f;
        }
        ekf.P[row][row] = EKF_INIT_P_POS;
    }
}

static void reset_slot_motion(int p)
{
    slot_vel_ema_h[p]       = 0.0f;
    slot_vel_ema_z[p]       = 0.0f;
    slot_hdg_valid[p]       = false;
    ekf.stationary_count[p] = 0;
}

static void alloc_slot(int slot, uint16_t id, int n_after)
{
    ekf.peer_ids[slot]        = id;
    ekf.peer_away[slot]       = false;
    ekf.peer_seeded[slot]     = false;
    ekf.peer_imu_valid[slot]  = false;

    /* Anchors: use the hardcoded hint table — position is known and fixed.
     * Tags:    use Xorshift32 RNG — position is unknown, random seed avoids
     *          the degenerate all-zeros start without implying a wrong location. */
    {
        const ekf_node_hint_t *hint = NULL;
        for (unsigned _h = 0; _h < EKF_POS_HINTS_N; _h++) {
            if (ekf_pos_hints[_h].id == id) { hint = &ekf_pos_hints[_h]; break; }
        }
        if (hint != NULL) {
            /* Known anchor — place at hint and mark seeded immediately.
             * This prevents seed_slot_centered() from overwriting the hint
             * with a random sphere placement when the first range arrives. */
            ekf.x[slot*3+0] = hint->x;
            ekf.x[slot*3+1] = hint->y;
            ekf.x[slot*3+2] = hint->z;
            ekf.peer_seeded[slot]    = true;
            ekf.peer_imu_valid[slot] = true;
            slot_last_x[slot]        = hint->x;
            slot_last_y[slot]        = hint->y;
        } else {
            /* Unknown tag — random position inside the convex hull of anchors.
             * Spread is EKF_INIT_RAND_M in XY, 0..EKF_INIT_RAND_M in Z. */
            uint32_t rng = (uint32_t)(HAL_GetTick()
                          ^ (uint32_t)id
                          ^ ((uint32_t)(unsigned)slot * 2654435761UL));
            rng ^= rng << 13; rng ^= rng >> 17; rng ^= rng << 5;
            ekf.x[slot*3+0] = ((float)(rng & 0xFFFFu) / 32767.5f - 1.0f) * EKF_INIT_RAND_M;
            rng ^= rng << 13; rng ^= rng >> 17; rng ^= rng << 5;
            ekf.x[slot*3+1] = ((float)(rng & 0xFFFFu) / 32767.5f - 1.0f) * EKF_INIT_RAND_M;
            rng ^= rng << 13; rng ^= rng >> 17; rng ^= rng << 5;
            ekf.x[slot*3+2] = ((float)(rng & 0xFFFFu) / 65535.0f) * EKF_INIT_RAND_M;
        }
    }
    mprintf("[EKF] alloc slot=%d id=0x%04X %s pos=(%.2f, %.2f, %.2f)\n",
            slot, (unsigned)id,
            ekf.peer_seeded[slot] ? "hint" : "rng ",
            (double)ekf.x[slot*3+0], (double)ekf.x[slot*3+1], (double)ekf.x[slot*3+2]);

    ekf.Q[slot*3+2][slot*3+2] = EKF_Q_Z_POS;
    reset_slot_motion(slot);
    reset_slot_covariance(slot, n_after);
}

/**
 * slot_reconnect
 *   Called when a previously-away peer returns.
 *   Preserves the last known position (x[]) and seeded flag — the existing
 *   estimate is always a better start than a fresh random seed.
 *   Inflates P so incoming measurements are trusted and converge quickly.
 */
static void slot_reconnect(int p)
{
    mprintf("[EKF] reconnect slot=%d id=0x%04X last_pos=(%.2f, %.2f, %.2f)\n",
            p, (unsigned)ekf.peer_ids[p],
            (double)ekf.x[p*3+0], (double)ekf.x[p*3+1], (double)ekf.x[p*3+2]);
    ekf.peer_away[p]      = false;
    ekf.peer_imu_valid[p] = false;          /* wait for a fresh IMU packet */
    reset_slot_motion(p);                   /* clear stale EMA / heading   */
    reset_slot_covariance(p, (int)ekf.n_peers * 3); /* inflate P, zero cross-terms */
    /* peer_seeded[p] and x[p*3..] intentionally preserved. */
}

/** Reset all per-slot motion arrays to zero. */
static void reset_motion_arrays(void)
{
    for (int p = 0; p < EKF_MAX_PEERS; p++) {
        slot_vel_ema_h[p]      = 0.0f;
        slot_vel_ema_z[p]      = 0.0f;
        slot_last_x[p]         = 0.0f;
        slot_last_y[p]         = 0.0f;
        slot_hdg_x[p]          = 1.0f;   /* unit-x default */
        slot_hdg_y[p]          = 0.0f;
        slot_hdg_valid[p]      = false;
        }
}


/* ============================================================================
 * Seeding
 * ============================================================================ */

/**
 * seed_slot_centered
 *   Places slot p at a random point on a sphere of radius range_m centred
 *   at (cx, cy, cz).  Elevation angle uniformly distributed in [0°, 26.6°]
 *   to avoid always seeding at Z = 0.
 */
static void seed_slot_centered(int p,
                                float cx, float cy, float cz,
                                float range_m)
{
    /* Xorshift32 seeded from tick + peer ID + slot index. */
    uint32_t rng = (uint32_t)(HAL_GetTick()
                 ^ (uint32_t)ekf.peer_ids[p]
                 ^ ((uint32_t)(unsigned)p * 2654435761UL));
    rng ^= rng << 13; rng ^= rng >> 17; rng ^= rng << 5;

    float seed_angle = (float)rng * 1.46291808e-9f;   /* maps to [0, 2π) */

    uint32_t rng2 = rng ^ 0xDEADBEEFUL;
    rng2 ^= rng2 << 13; rng2 ^= rng2 >> 17; rng2 ^= rng2 << 5;
    float elev_frac = (float)(rng2 & 0xFFu) / 255.0f;  /* 0..1            */
    float z_frac    = elev_frac * 0.5f;                 /* max half-range  */
    float xy_r      = sqrtf(fmaxf(0.0f,
                        range_m * range_m
                      - (range_m * z_frac) * (range_m * z_frac)));

    ekf.x[p*3+0] = cx + xy_r * cosf(seed_angle);
    ekf.x[p*3+1] = cy + xy_r * sinf(seed_angle);
    ekf.x[p*3+2] = cz + range_m * z_frac;

    ekf.peer_seeded[p]    = true;
    ekf.peer_imu_valid[p] = true;
    slot_last_x[p]        = ekf.x[p*3+0];
    slot_last_y[p]        = ekf.x[p*3+1];
    slot_hdg_valid[p]     = false;
    mprintf("[EKF] seeded slot=%d id=0x%04X pos=(%.2f, %.2f, %.2f) r=%.2f\n",
            p, (unsigned)ekf.peer_ids[p],
            (double)ekf.x[p*3+0], (double)ekf.x[p*3+1], (double)ekf.x[p*3+2],
            (double)range_m);
}

/** Convenience wrapper: seed around world origin (used in Phase 2 from anchor 0). */

/* ============================================================================
 * apply_range_update_ext
 * ─────────────────────────────────────────────────────────────────────────────
 * Sequential EKF update for one scalar range measurement between two endpoints.
 *
 *   fixed_i  : if non-NULL, endpoint i is at this fixed position (not in state).
 *   base_i   : state base index for i; ignored when fixed_i != NULL.
 *   fixed_j  : if non-NULL, endpoint j is at this fixed position.
 *   base_j   : state base index for j; ignored when fixed_j != NULL.
 *   range_m  : measured distance (m).
 *   sigma_d  : measurement noise std-dev (m).
 *   n        : active state dimension.
 *
 * Covers all cases in one function:
 *   fixed anchor → estimated tag  :  fixed_i = anchor_pos[a],  fixed_j = NULL
 *   estimated    → estimated      :  fixed_i = NULL,            fixed_j = NULL
 *   origin       → estimated      :  fixed_i = k_origin,        fixed_j = NULL
 *
 * Covariance update uses the Joseph form to preserve symmetry and PSD.
 * ============================================================================ */
static void apply_range_update_ext(const float *fixed_i, int base_i,
                                    const float *fixed_j, int base_j,
                                    float range_m, float sigma_d, int n)
{
    float xi = fixed_i ? fixed_i[0] : ekf.x[base_i + 0];
    float yi = fixed_i ? fixed_i[1] : ekf.x[base_i + 1];
    float zi = fixed_i ? fixed_i[2] : ekf.x[base_i + 2];
    float xj = fixed_j ? fixed_j[0] : ekf.x[base_j + 0];
    float yj = fixed_j ? fixed_j[1] : ekf.x[base_j + 1];
    float zj = fixed_j ? fixed_j[2] : ekf.x[base_j + 2];

    float dx = xj - xi, dy = yj - yi, dz = zj - zi;
    float h  = sqrtf(dx*dx + dy*dy + dz*dz);
    if (h < 1.0e-4f) h = 1.0e-4f;

    float ux = dx/h, uy = dy/h, uz = dz/h;

    float H[EKF_MAX_STATE];
    memset(H, 0, sizeof(H));
    if (!fixed_i) { H[base_i+0] = -ux; H[base_i+1] = -uy; H[base_i+2] = -uz; }
    if (!fixed_j) { H[base_j+0] = +ux; H[base_j+1] = +uy; H[base_j+2] = +uz; }

    /* PH = P · H  (exploit sparsity: only non-zero H entries matter) */
    float PH[EKF_MAX_STATE];
    for (int i = 0; i < n; i++) {
        float acc = 0.0f;
        for (int j = 0; j < n; j++) acc += ekf.P[i][j] * H[j];
        PH[i] = acc;
    }

    /* Innovation variance  S = H·P·H^T + R */
    float S = sigma_d * sigma_d;
    for (int j = 0; j < n; j++) S += H[j] * PH[j];

    /* Outlier gate */
    float innov = range_m - h;
    if (innov >  EKF_OUTLIER_GATE_POS * sqrtf(S)) return;
    if (innov < -EKF_OUTLIER_GATE_NEG * sqrtf(S)) return;

    /* Kalman gain  K = PH / S */
    float S_inv = 1.0f / S;
    float K[EKF_MAX_STATE];
    for (int i = 0; i < n; i++) K[i] = PH[i] * S_inv;

    /* State update  x += K · innov */
    for (int i = 0; i < n; i++) ekf.x[i] += K[i] * innov;

    /* Per-slot displacement clamp — limits how far any single update can
     * move a node.  Protects against sudden measurement outliers that slip
     * through the gate during large initial uncertainty.                   */
    for (int p_s = 0; p_s < n / 3; p_s++) {
        int   b  = p_s * 3;
        float cx = K[b+0] * innov;
        float cy = K[b+1] * innov;
        float cz = K[b+2] * innov;
        float cm = sqrtf(cx*cx + cy*cy + cz*cz);
        if (cm > EKF_MAX_UPDATE_M) {
            float scale = EKF_MAX_UPDATE_M / cm;
            ekf.x[b+0] -= cx * (1.0f - scale);
            ekf.x[b+1] -= cy * (1.0f - scale);
            ekf.x[b+2] -= cz * (1.0f - scale);
        }
    }

    /* Joseph-form covariance update  P = P - K·PH^T - PH·K^T + S·K·K^T
     * Preserves symmetry exactly under finite-precision arithmetic.       */
    for (int i = 0; i < n; i++) {
        for (int j = i; j < n; j++) {
            float v = ekf.P[i][j]
                    - K[i] * PH[j]
                    - PH[i] * K[j]
                    + S * K[i] * K[j];
            ekf.P[i][j] = ekf.P[j][i] = v;
        }
    }
}


/* ============================================================================
 * predict_anchor_slot  (Phase 1 only)
 * ─────────────────────────────────────────────────────────────────────────────
 * Anchors are stationary: no position update to x[].
 * A tiny process noise is added to P to maintain numerical health; it is
 * negligible compared to the range-update information rate.
 * ============================================================================ */
static void predict_anchor_slot(int p, float dt_s)
{
    int   base = p * 3;
    float q    = EKF_Q_H_FLOOR_STATIONARY * (dt_s / EKF_DT_NOM_S);
    for (int k = 0; k < 3; k++) {
        ekf.P[base+k][base+k] += q;
        if (ekf.P[base+k][base+k] > EKF_P_MAX)
            ekf.P[base+k][base+k] = EKF_P_MAX;
    }
}


/* ============================================================================
 * predict_slot  (Phase 2, unchanged from v1)
 * ============================================================================ */
static void predict_slot(int p, float vh_raw, float vz_raw, float dt_s)
{
    int base = p * 3;

    float pre_x = ekf.x[base+0];
    float pre_y = ekf.x[base+1];
    float pre_z = ekf.x[base+2];

    /* Tag is always treated as moving — no stationary detection.
     * IMU deadband still applied to suppress sensor noise floor. */
    float vh_eff = (fabsf(vh_raw) < EKF_IMU_DEADBAND_MS) ? 0.0f : vh_raw;
    float vz_eff = (fabsf(vz_raw) < EKF_IMU_DEADBAND_MS) ? 0.0f : vz_raw;

    slot_vel_ema_h[p] = EKF_VEL_EMA_ALPHA * vh_eff
                      + (1.0f - EKF_VEL_EMA_ALPHA) * slot_vel_ema_h[p];
    slot_vel_ema_z[p] = EKF_VEL_EMA_ALPHA * vz_eff
                      + (1.0f - EKF_VEL_EMA_ALPHA) * slot_vel_ema_z[p];

    ekf.x[base + 2] += slot_vel_ema_z[p] * dt_s;

    if (slot_hdg_valid[p] && slot_vel_ema_h[p] > EKF_IMU_DEADBAND_MS) {
        ekf.x[base + 0] += slot_hdg_x[p] * slot_vel_ema_h[p] * dt_s;
        ekf.x[base + 1] += slot_hdg_y[p] * slot_vel_ema_h[p] * dt_s;
    }

    /* Max-displacement clamp */
    float ddx  = ekf.x[base+0] - pre_x;
    float ddy  = ekf.x[base+1] - pre_y;
    float ddz  = ekf.x[base+2] - pre_z;
    float dist = sqrtf(ddx*ddx + ddy*ddy + ddz*ddz);
    if (dist > EKF_MAX_STEP_M) {
        float scale = EKF_MAX_STEP_M / dist;
        ekf.x[base+0] = pre_x + ddx * scale;
        ekf.x[base+1] = pre_y + ddy * scale;
        ekf.x[base+2] = pre_z + ddz * scale;
    }

    /* Process noise — always moving floor, no stationary branch */
    float v_clamped = fmaxf(slot_vel_ema_h[p], 0.0f);
    float h_var = v_clamped * v_clamped * dt_s * dt_s * EKF_Q_H_VEL_SCALE
                + EKF_Q_H_FLOOR * (dt_s / EKF_DT_NOM_S);

    float z_var = EKF_Q_Z_POS * (dt_s / EKF_DT_NOM_S);

    ekf.Q[base+0][base+0] = h_var;
    ekf.Q[base+1][base+1] = h_var;
    ekf.Q[base+2][base+2] = z_var;

    ekf.P[base+0][base+0] += h_var;
    if (ekf.P[base+0][base+0] > EKF_P_MAX) ekf.P[base+0][base+0] = EKF_P_MAX;
    ekf.P[base+1][base+1] += h_var;
    if (ekf.P[base+1][base+1] > EKF_P_MAX) ekf.P[base+1][base+1] = EKF_P_MAX;
    ekf.P[base+2][base+2] += z_var;
    if (ekf.P[base+2][base+2] > EKF_P_MAX) ekf.P[base+2][base+2] = EKF_P_MAX;

    /* Cauchy-Schwarz PSD clamp on the 3×3 self-block */
    for (int r = 0; r < 3; r++) {
        for (int c = r + 1; c < 3; c++) {
            float lim = sqrtf(ekf.P[base+r][base+r] * ekf.P[base+c][base+c]);
            if (ekf.P[base+r][base+c] >  lim) ekf.P[base+r][base+c] =  lim;
            if (ekf.P[base+r][base+c] < -lim) ekf.P[base+r][base+c] = -lim;
            ekf.P[base+c][base+r] = ekf.P[base+r][base+c];
        }
    }
}


/* ============================================================================
 * Phase 2 network write-back helpers
 * ============================================================================ */
static void write_slot_position_to_network(int p, network_t *net)
{
    if ((net == NULL) || (!ekf.peer_seeded[p])) {
        return;
    }

    for (int k = 0; k < (int)net->count; k++) {
        if (net->peers[k].id == ekf.peer_ids[p]) {
            net->peers[k].pos[0] = ekf.x[p*3+0];
            net->peers[k].pos[1] = ekf.x[p*3+1];
            net->peers[k].pos[2] = ekf.x[p*3+2];
            break;
        }
    }
}

static void update_heading_and_writeback(int p, network_t *net)
{
    if (!ekf.peer_seeded[p]) return;

    float dx   = ekf.x[p*3+0] - slot_last_x[p];
    float dy   = ekf.x[p*3+1] - slot_last_y[p];
    float dmag = sqrtf(dx*dx + dy*dy);
    if (dmag >= EKF_HDG_MIN_DELTA_M) {
        slot_hdg_x[p]     = dx / dmag;
        slot_hdg_y[p]     = dy / dmag;
        slot_hdg_valid[p] = true;
    }
    slot_last_x[p] = ekf.x[p*3+0];
    slot_last_y[p] = ekf.x[p*3+1];

    write_slot_position_to_network(p, net);
}


/* ============================================================================
 * ╔══════════════════════════════════════════════════════════════════════════╗
 * ║                       PHASE 1 — ANCHOR SURVEY                          ║
 * ╚══════════════════════════════════════════════════════════════════════════╝
 * ============================================================================ */

/**
 * sync_peers_phase1
 *   Reconciles the EKF slot table with the network for Phase 1.
 *   Only non-origin anchor IDs get (or retain) slots.
 *   Non-anchor (tag) peers are completely ignored.
 *
 *   Disconnect rule  : mark away, reset P (inflate), keep position.
 *   Reconnect rule   : clear away flag, inflate P, keep position (slot_reconnect).
 *   New anchor rule  : allocate slot if not yet seen and present in network.
 */
static void sync_peers_phase1(void)
{
    network_t *net   = network_get_network();
    uint16_t  own_id = network_get_ownid();
    int       n      = (int)ekf.n_peers * 3;

    /* Rules 1 & 2 — existing anchor slots */
    for (int p = 0; p < (int)ekf.n_peers; p++) {
        if (p == ekf.self_slot) continue;   /* self managed at init */

        bool in_net = false;
        for (int k = 0; k < (int)net->count; k++)
            if (net->peers[k].id == ekf.peer_ids[p]) { in_net = true; break; }

        if (!in_net) {
            if (!ekf.peer_away[p]) {
                ekf.peer_away[p] = true;
                reset_slot_motion(p);
                reset_slot_covariance(p, n);
            }
        } else if (ekf.peer_away[p]) {
            slot_reconnect(p);
        }
    }

    /* Rule 3 — allocate slots for newly-seen non-origin anchors */
    for (int a = 1; a < (int)EKF_NUM_ANCHORS; a++) {
        uint16_t aid = EKF_ANCHOR_IDS[a];
        if (aid == own_id)           continue;   /* self handled at init */
        if (find_ekf_slot(aid) >= 0) continue;   /* already has a slot   */

        /* Check network */
        bool in_net = false;
        for (int k = 0; k < (int)net->count; k++)
            if (net->peers[k].id == aid) { in_net = true; break; }
        if (!in_net)                         continue;
        if (ekf.n_peers >= EKF_MAX_PEERS)    continue;

        int slot = (int)ekf.n_peers++;
        alloc_slot(slot, aid, (int)ekf.n_peers * 3);
    }
}

/**
 * phase1_all_anchors_present
 *   Returns true when all four anchor IDs have been seen and are not away.
 */
static bool phase1_all_anchors_present(void)
{
    uint16_t own_id = network_get_ownid();

    for (int a = 0; a < (int)EKF_NUM_ANCHORS; a++) {
        int8_t slot;

        if (EKF_ANCHOR_IDS[a] == own_id) {
            continue;   /* self is always locally present */
        }

        if (a == 0) {
            /* Anchor 0 is a fixed reference and intentionally has no EKF slot.
             * Its current presence must therefore be checked in the network. */
            if (!network_contains_id(EKF_ANCHOR_IDS[a])) {
                return false;
            }
            continue;
        }

        slot = find_ekf_slot(EKF_ANCHOR_IDS[a]);
        if (slot < 0) {
            return false;       /* never seen */
        }
        if (ekf.peer_away[slot]) {
            return false;       /* currently absent */
        }
    }

    return true;
}

/**
 * phase1_converged
 *   Returns true when:
 *     1. All four anchors are present.
 *     2. Every estimated anchor axis has P[k][k] < EKF_ANCHOR_P_CONVERGED.
 *     3. At least EKF_ANCHOR_MIN_STEPS have been executed.
 */
static bool phase1_converged(void)
{
    if (phase1_steps < EKF_ANCHOR_MIN_STEPS) return false;
    if (!phase1_all_anchors_present())        return false;

    for (int p = 0; p < (int)ekf.n_peers; p++) {
        if (!ekf.peer_seeded[p]) return false;
        for (int k = 0; k < 3; k++)
            if (ekf.P[p*3+k][p*3+k] >= EKF_ANCHOR_P_CONVERGED) return false;
    }
    return true;
}


/* ============================================================================
 * ╔══════════════════════════════════════════════════════════════════════════╗
 * ║                PHASE 1 → PHASE 2  TRANSITION                           ║
 * ╚══════════════════════════════════════════════════════════════════════════╝
 * ============================================================================ */

static void ekf_init_phase2(void);   /* forward declaration */

/**
 * transition_to_phase2
 *   1.  Freezes the four anchor positions from the Phase 1 EKF result.
 *   2.  Toggles LED_R once to signal successful anchor survey.
 *   3.  Re-initialises the EKF for Phase 2 tag tracking.
 */
static void transition_to_phase2(void)
{
    uint16_t own_id = network_get_ownid();

    /* ── 1. Freeze anchor 0 at world origin ────────────────────────────── */
    anchor_pos[0][0] = 0.0f;
    anchor_pos[0][1] = 0.0f;
    anchor_pos[0][2] = 0.0f;

    /* ── 2. Copy estimated positions for anchors 1–3 ───────────────────── */
    for (int p = 0; p < (int)ekf.n_peers; p++) {
        uint16_t pid = (p == ekf.self_slot) ? own_id : ekf.peer_ids[p];
        int ai = anchor_index_of(pid);
        if (ai <= 0) continue;          /* skip origin (0) and non-anchors  */

        anchor_pos[ai][0] = ekf.x[p*3+0];
        anchor_pos[ai][1] = ekf.x[p*3+1];
        anchor_pos[ai][2] = ekf.x[p*3+2];
    }

    /* ── 3. Signal success ──────────────────────────────────────────────── */
    mprintf("[EKF] phase1->phase2 after %lu steps anchors: "
            "(%.2f,%.2f,%.2f) (%.2f,%.2f,%.2f) (%.2f,%.2f,%.2f) (%.2f,%.2f,%.2f)\n",
            (unsigned long)phase1_steps,
            (double)anchor_pos[0][0],(double)anchor_pos[0][1],(double)anchor_pos[0][2],
            (double)anchor_pos[1][0],(double)anchor_pos[1][1],(double)anchor_pos[1][2],
            (double)anchor_pos[2][0],(double)anchor_pos[2][1],(double)anchor_pos[2][2],
            (double)anchor_pos[3][0],(double)anchor_pos[3][1],(double)anchor_pos[3][2]);
    HAL_GPIO_TogglePin(LED_R_GPIO_Port, LED_R_Pin);
    ekf_phase = EKF_PHASE_TAG_LOCALIZE;

    /* ── 4. Re-initialise filter for Phase 2 ───────────────────────────── */
    ekf_init_phase2();
}


/* ============================================================================
 * ekf_step_phase1
 * ─────────────────────────────────────────────────────────────────────────────
 * One step of the Phase 1 anchor-survey EKF.
 *
 * PREDICT : tiny stationary noise only (no motion update).
 * UPDATE  : all available inter-anchor range pairs.
 *           Uses only the four hardcoded anchor IDs as measurement sources.
 *           Non-anchor nodes in the network are ignored.
 * ============================================================================ */
static void ekf_step_phase1(float dt_s)
{
    sync_peers_phase1();
    int n = (int)ekf.n_peers * 3;

    /* ── PREDICT ─────────────────────────────────────────────────────────── */
    for (int p = 0; p < (int)ekf.n_peers; p++) {
        if (ekf.peer_away[p]) continue;
        predict_anchor_slot(p, dt_s);
    }

    /* ── UPDATE — all anchor pairs (ai < aj) ─────────────────────────────── */
    for (int ai = 0; ai < (int)EKF_NUM_ANCHORS; ai++) {
        for (int aj = ai + 1; aj < (int)EKF_NUM_ANCHORS; aj++) {

            int slot_i = anchor_idx_to_slot(ai);
            int slot_j = anchor_idx_to_slot(aj);

            /* Skip if a non-origin anchor has no slot yet or is absent */
            if (ai > 0 && (slot_i < 0 || ekf.peer_away[slot_i])) continue;
            if (aj > 0 && (slot_j < 0 || ekf.peer_away[slot_j])) continue;

            /* Fetch range measurement.  The network layer may store the pair
             * in either orientation; the EKF treats range as symmetric. */
            node_peer_state_t *ps =
                get_peer_state_symmetric(EKF_ANCHOR_IDS[ai], EKF_ANCHOR_IDS[aj]);
            if (ps == NULL) {
                continue;
            }

            float range_m = range_scaled_to_m(ps->distance_scaled);
            if (range_m < 0.0f) {
                continue;
            }
            float sigma_d = ekf_certainty_to_sigma(ps->certainty);

            bool seeded_i = (ai == 0) || ekf.peer_seeded[slot_i];
            bool seeded_j = (aj == 0) || ekf.peer_seeded[slot_j];

            /* Seed only from an endpoint that is already known.  This is
             * required for Phase 1 because an unseeded x[] block still contains
             * zeros, which would otherwise look like a valid reference.
             *
             * IMPORTANT: if a slot was pre-seeded from ekf_pos_hints[] in
             * alloc_slot(), ekf.peer_seeded[] is already true — do NOT call
             * seed_slot_centered() for it, as that would overwrite the hint
             * with a random sphere placement and destroy the orientation fix. */
            if ((aj > 0) && (!seeded_j) && seeded_i) {
                if (ekf.peer_seeded[slot_j]) {
                    /* Already seeded from hint — just update local flag. */
                    seeded_j = true;
                } else {
                    float pi_x;
                    float pi_y;
                    float pi_z;

                    if (ai == 0) {
                        pi_x = 0.0f;
                        pi_y = 0.0f;
                        pi_z = 0.0f;
                    } else {
                        pi_x = ekf.x[(slot_i * 3) + 0];
                        pi_y = ekf.x[(slot_i * 3) + 1];
                        pi_z = ekf.x[(slot_i * 3) + 2];
                    }
                    seed_slot_centered(slot_j, pi_x, pi_y, pi_z, range_m);
                    seeded_j = true;
                }
            }

            if ((ai > 0) && (!seeded_i) && seeded_j) {
                if (ekf.peer_seeded[slot_i]) {
                    /* Already seeded from hint — just update local flag. */
                    seeded_i = true;
                } else {
                    float pj_x = ekf.x[(slot_j * 3) + 0];
                    float pj_y = ekf.x[(slot_j * 3) + 1];
                    float pj_z = ekf.x[(slot_j * 3) + 2];

                    seed_slot_centered(slot_i, pj_x, pj_y, pj_z, range_m);
                    seeded_i = true;
                }
            }

            /* Both must be seeded, or fixed at origin, before update. */
            if ((!seeded_i) || (!seeded_j)) {
                continue;
            }

            /* Apply EKF update */
            const float *fp_i = (ai == 0) ? k_origin : NULL;
            const float *fp_j = (aj == 0) ? k_origin : NULL;
            apply_range_update_ext(fp_i, (ai == 0) ? 0 : slot_i * 3,
                                   fp_j, (aj == 0) ? 0 : slot_j * 3,
                                   range_m, sigma_d, n);
        }
    }

    phase1_steps++;
    ekf.initialised = true;

    /* ── WRITE-BACK: publish own position if self is an anchor ─────────── */
    uint16_t own_id    = network_get_ownid();
    int      own_anch  = anchor_index_of(own_id);

    if (own_anch == 0) {
        /* Self is the origin anchor */
        publish_self_position(k_origin);
    } else if (own_anch > 0
               && ekf.self_slot >= 0
               && ekf.peer_seeded[ekf.self_slot]) {
        float sp[3] = { ekf.x[ekf.self_slot*3+0],
                        ekf.x[ekf.self_slot*3+1],
                        ekf.x[ekf.self_slot*3+2] };
        publish_self_position(sp);
    }
    /* Tag device: does not publish a position during Phase 1. */

    /* ── CONVERGENCE CHECK → trigger transition ─────────────────────────── */
    if (phase1_converged()) {
        mprintf("[EKF] phase1 converged after %lu steps\n",
                (unsigned long)phase1_steps);
        transition_to_phase2();
    }
}


/* ============================================================================
 * ╔══════════════════════════════════════════════════════════════════════════╗
 * ║                     PHASE 2 — TAG LOCALIZATION                         ║
 * ╚══════════════════════════════════════════════════════════════════════════╝
 * ============================================================================ */

/**
 * sync_peers_phase2
 *   Reconciles EKF slot table with network for Phase 2.
 *   Anchor IDs are SKIPPED — they are fixed reference points, not slots.
 *
 *   Disconnect / reconnect handling:
 *     Away   : P inflated, position preserved (stability over accuracy).
 *     Return : slot_reconnect() — position preserved, P inflated, IMU reset.
 */
static void sync_peers_phase2(void)
{
    network_t *net   = network_get_network();
    uint16_t  own_id = network_get_ownid();
    int       n      = (int)ekf.n_peers * 3;

    /* Rules 1 & 2 */
    for (int p = 0; p < (int)ekf.n_peers; p++) {
        if (p == ekf.self_slot)            continue;
        if (is_anchor_id(ekf.peer_ids[p])) continue;   /* anchors not in P2 state */

        bool in_net = false;
        for (int k = 0; k < (int)net->count; k++)
            if (net->peers[k].id == ekf.peer_ids[p]) { in_net = true; break; }

        if (!in_net) {
            if (!ekf.peer_away[p]) {
                ekf.peer_away[p] = true;
                reset_slot_motion(p);
                reset_slot_covariance(p, n);
                /* x[p*3..] and peer_seeded[p] preserved — last known position kept */
            }
        } else if (ekf.peer_away[p]) {
            slot_reconnect(p);   /* preserves position, inflates P */
        }
    }

    /* Rule 3 — allocate slots for newly-seen non-anchor peers */
    for (int k = 0; k < (int)net->count; k++) {
        uint16_t pid = net->peers[k].id;
        if (pid == 0 || pid == own_id)    continue;
        if (is_anchor_id(pid))             continue;   /* anchors not tracked */
        if (find_ekf_slot(pid) >= 0)       continue;
        if (ekf.n_peers >= EKF_MAX_PEERS)  continue;

        int slot = (int)ekf.n_peers++;
        alloc_slot(slot, pid, (int)ekf.n_peers * 3);
    }
}

/**
 * ekf_init_phase2
 *   Called from transition_to_phase2().
 *   Clears the EKF struct and allocates slots for all non-anchor devices
 *   currently visible in the network.
 */
static void ekf_init_phase2(void)
{
    mprintf("[EKF] ekf_init_phase2 called\n");
    memset(&ekf, 0, sizeof(ekf));
    reset_motion_arrays();

    ekf.self_slot    = -1;
    ekf.initialised  = false;
    ekf.last_tick_ms = HAL_GetTick();

    network_t *net   = network_get_network();
    uint16_t  own_id = network_get_ownid();

    /* Self slot — only allocated if self is a tag (not an anchor) */
    if (!is_anchor_id(own_id)) {
        int slot      = (int)ekf.n_peers++;
        ekf.self_slot = (int8_t)slot;
        alloc_slot(slot, own_id, (int)ekf.n_peers * 3);
        ekf.peer_imu_valid[slot] = true;   /* self IMU always valid */
    }

    /* Peer slots for all non-anchor devices currently in network */
    for (int i = 0; i < (int)net->count
                 && (int)ekf.n_peers < EKF_MAX_PEERS; i++) {
        uint16_t pid = net->peers[i].id;
        if (pid == 0 || pid == own_id) continue;
        if (is_anchor_id(pid))         continue;
        if (find_ekf_slot(pid) >= 0)   continue;

        int slot = (int)ekf.n_peers++;
        alloc_slot(slot, pid, (int)ekf.n_peers * 3);
    }

    int n = (int)ekf.n_peers * 3;
    for (int i = 0; i < n; i++) ekf.P[i][i] = EKF_INIT_P_POS;
    for (int p = 0; p < (int)ekf.n_peers; p++)
        ekf.Q[p*3+2][p*3+2] = EKF_Q_Z_POS;
}

/**
 * ekf_step_phase2
 *   One step of the Phase 2 tag-localization EKF.
 *
 *   PREDICT  : IMU-driven for each tag slot.
 *   UPDATE 1 : anchor → tag  (fixed anchor position from anchor_pos[]).
 *   UPDATE 2 : tag   → tag   (both in state; standard peer–peer update).
 *   WRITE-BACK: tag positions → network peer table;
 *               anchor positions → network peer table (for external consumers);
 *               self position  → network_set_self_pos().
 */
static void ekf_step_phase2(float az_self_ms, float ah_self_ms, float dt_s)
{
    sync_peers_phase2();
    int      n      = (int)ekf.n_peers * 3;
    uint16_t own_id = network_get_ownid();

    /* ── PREDICT ─────────────────────────────────────────────────────────── */
    for (int p = 0; p < (int)ekf.n_peers; p++) {
        if (ekf.peer_away[p]) continue;

        float vh_raw, vz_raw;
        if (p == ekf.self_slot) {
            vh_raw = ah_self_ms;
            vz_raw = az_self_ms;
        } else {
            node_t *peer_node;

            vh_raw = 0.0f;
            vz_raw = 0.0f;

            peer_node = find_peer(ekf.peer_ids[p]);
            if (peer_node != NULL) {
                vz_raw = vel_vert_u8_to_ms(peer_node->imu_vel_vert);
                vh_raw = vel_horiz_u8_to_ms(peer_node->imu_vel_horiz);
                ekf.peer_imu_valid[p] = true;
            } else {
                ekf.peer_imu_valid[p] = false;
            }
        }
        predict_slot(p, vh_raw, vz_raw, dt_s);
    }

    /* ── UPDATE 1: anchor → tag ──────────────────────────────────────────── */
    for (int p = 0; p < (int)ekf.n_peers; p++) {
        if (ekf.peer_away[p]) continue;
        uint16_t pid = (p == ekf.self_slot) ? own_id : ekf.peer_ids[p];

        for (int a = 0; a < (int)EKF_NUM_ANCHORS; a++) {
            /* Query the stored measurement between this anchor and this tag */
            node_peer_state_t *ps = get_peer_state_symmetric(EKF_ANCHOR_IDS[a], pid);
            if (ps == NULL) {
                continue;
            }

            float range_m = range_scaled_to_m(ps->distance_scaled);
            if (range_m < 0.0f) {
                continue;
            }

            /* Seed from the first available fixed anchor.  Anchor 0 is tried
             * first by loop order, but a valid range to A1..A3 is also enough. */
            if (!ekf.peer_seeded[p]) {
                mprintf("[EKF] phase2 seed slot=%d from anchor=%d range=%.2f\n",
                        p, a, (double)range_m);
                seed_slot_centered(p, anchor_pos[a][0], anchor_pos[a][1],
                                   anchor_pos[a][2], range_m);
            }

            float sigma_d = ekf_certainty_to_sigma(ps->certainty);
            apply_range_update_ext(anchor_pos[a], 0,     /* fixed anchor   */
                                   NULL, p * 3,           /* tag in state   */
                                   range_m, sigma_d, n);
        }
    }

    /* ── UPDATE 2: tag → tag ─────────────────────────────────────────────── */
    for (int pi = 0; pi < (int)ekf.n_peers; pi++) {
        if (ekf.peer_away[pi] || !ekf.peer_seeded[pi]) continue;

        for (int pj = pi + 1; pj < (int)ekf.n_peers; pj++) {
            if (ekf.peer_away[pj] || !ekf.peer_seeded[pj]) continue;

            uint16_t id_i = (pi == ekf.self_slot) ? own_id : ekf.peer_ids[pi];
            uint16_t id_j = (pj == ekf.self_slot) ? own_id : ekf.peer_ids[pj];
            node_peer_state_t *ps = get_peer_state_symmetric(id_i, id_j);

            if (ps == NULL) {
                continue;
            }

            float range_m = range_scaled_to_m(ps->distance_scaled);
            if (range_m < 0.0f) {
                continue;
            }

            float sigma_d = ekf_certainty_to_sigma(ps->certainty);
            apply_range_update_ext(NULL, pi * 3,
                                   NULL, pj * 3,
                                   range_m, sigma_d, n);
        }
    }

    ekf.initialised = true;

    /* ── WRITE-BACK ──────────────────────────────────────────────────────── */
    network_t *net = network_get_network();

    /* Publish frozen anchor positions into the network peer table so that
     * any other firmware module that reads peer.pos gets correct values.  */
    for (int a = 0; a < (int)EKF_NUM_ANCHORS; a++) {
        for (int k = 0; k < (int)net->count; k++) {
            if (net->peers[k].id == EKF_ANCHOR_IDS[a]) {
                net->peers[k].pos[0] = anchor_pos[a][0];
                net->peers[k].pos[1] = anchor_pos[a][1];
                net->peers[k].pos[2] = anchor_pos[a][2];
                break;
            }
        }
    }

    /* Publish estimated tag positions for active peers only.
     * The network layer treats net->peers[] as the active snapshot and removes
     * disconnected peers.  Away-tag positions are preserved inside the EKF and
     * are available locally via ekf_get_peer_pos(); they are not written into
     * net->peers[] because there is no active row for them. */
    for (int p = 0; p < (int)ekf.n_peers; p++) {
        if (p == ekf.self_slot) {
            if (ekf.peer_seeded[p]) {
                float sp[3] = { ekf.x[p*3+0], ekf.x[p*3+1], ekf.x[p*3+2] };
                publish_self_position(sp);
                /* Update heading from position delta */
                float dx   = ekf.x[p*3+0] - slot_last_x[p];
                float dy   = ekf.x[p*3+1] - slot_last_y[p];
                float dmag = sqrtf(dx*dx + dy*dy);
                if (dmag >= EKF_HDG_MIN_DELTA_M) {
                    slot_hdg_x[p]     = dx / dmag;
                    slot_hdg_y[p]     = dy / dmag;
                    slot_hdg_valid[p] = true;
                }
                slot_last_x[p] = ekf.x[p*3+0];
                slot_last_y[p] = ekf.x[p*3+1];
            }
        } else if (!ekf.peer_away[p]) {
            update_heading_and_writeback(p, net);
        } else {
            /* Local-only stale position: preserved in EKF state, not published. */
        }
    }

    /* If self is an anchor in Phase 2, publish its frozen position. */
    int own_anch = anchor_index_of(own_id);
    if (own_anch >= 0)
        publish_self_position(anchor_pos[own_anch]);
}


/* ============================================================================
 * ╔══════════════════════════════════════════════════════════════════════════╗
 * ║                          PUBLIC API                                     ║
 * ╚══════════════════════════════════════════════════════════════════════════╝
 * ============================================================================ */

void ekf_init(void)
{
    /* A real MCU hard reset clears static RAM before this function is called.
     * If the network/application layer calls ekf_init() during a Phase 2
     * self-disconnect/reconnect event, do not erase the solved anchor map or
     * last-known tag positions.  Only reset stale motion priors and inflate P
     * so fresh UWB data can pull the state back in cleanly. */
    if (ekf_phase == EKF_PHASE_TAG_LOCALIZE) {
        int n = (int)ekf.n_peers * 3;
        uint16_t own_id = network_get_ownid();

        reset_motion_arrays();
        for (int p = 0; p < (int)ekf.n_peers; p++) {
            reset_slot_motion(p);
            reset_slot_covariance(p, n);
            ekf.peer_imu_valid[p] = (ekf.peer_ids[p] == own_id);
        }
        ekf.last_tick_ms = HAL_GetTick();
        return;
    }

    memset(&ekf, 0, sizeof(ekf));
    memset(anchor_pos, 0, sizeof(anchor_pos));
    reset_motion_arrays();

    ekf_phase     = EKF_PHASE_ANCHOR_SURVEY;
    phase1_steps  = 0;

    ekf.self_slot    = -1;
    ekf.initialised  = false;
    ekf.last_tick_ms = HAL_GetTick();

    network_t *net   = network_get_network();
    uint16_t  own_id = network_get_ownid();

    /* Determine if self is one of the anchors */
    int self_anchor_idx = anchor_index_of(own_id);

    if (self_anchor_idx > 0) {
        /* Self is a non-origin anchor → allocate self_slot for Phase 1 */
        int slot      = (int)ekf.n_peers++;
        ekf.self_slot = (int8_t)slot;
        alloc_slot(slot, own_id, (int)ekf.n_peers * 3);
        /* Anchors are stationary — we set imu_valid false so predict_slot
         * never uses stale IMU data if accidentally called. */
        ekf.peer_imu_valid[slot] = false;
    }
    /* self_anchor_idx == 0  : self is origin anchor, no slot needed.       */
    /* self_anchor_idx == -1 : self is a tag, no slot in Phase 1.           */

    /* Allocate slots for non-origin anchor peers currently in network */
    for (int a = 1; a < (int)EKF_NUM_ANCHORS; a++) {
        uint16_t aid = EKF_ANCHOR_IDS[a];
        if (aid == own_id)           continue;   /* already handled above  */
        if (find_ekf_slot(aid) >= 0) continue;

        for (int k = 0; k < (int)net->count; k++) {
            if (net->peers[k].id == aid) {
                if (ekf.n_peers >= EKF_MAX_PEERS) break;
                int slot = (int)ekf.n_peers++;
                alloc_slot(slot, aid, (int)ekf.n_peers * 3);
                break;
            }
        }
    }

    int n = (int)ekf.n_peers * 3;
    for (int i = 0; i < n; i++) ekf.P[i][i] = EKF_INIT_P_POS;
    for (int p = 0; p < (int)ekf.n_peers; p++)
        ekf.Q[p*3+2][p*3+2] = EKF_Q_Z_POS;
}


void ekf_step(float az_self_ms, float ah_self_ms)
{
    /* ── Variable dt ─────────────────────────────────────────────────────── */
    uint32_t now_ms = HAL_GetTick();
    float dt_s;
    if (!ekf.initialised) {
        dt_s = EKF_DT_NOM_S;
    } else {
        dt_s = (float)((uint32_t)(now_ms - ekf.last_tick_ms)) * 0.001f;
        if (dt_s < 0.001f)       dt_s = EKF_DT_NOM_S;
        if (dt_s > EKF_DT_MAX_S) dt_s = EKF_DT_MAX_S;
    }
    ekf.last_tick_ms = now_ms;

    /* ── Dispatch to current phase ───────────────────────────────────────── */
    if (ekf_phase == EKF_PHASE_ANCHOR_SURVEY)
        ekf_step_phase1(dt_s);
    else
        ekf_step_phase2(az_self_ms, ah_self_ms, dt_s);
}


bool ekf_anchor_ready(void)
{
    return (ekf_phase == EKF_PHASE_TAG_LOCALIZE);
}

ekf_phase_t ekf_get_phase(void)
{
    return ekf_phase;
}

const float (*ekf_get_anchor_positions(void))[3]
{
    return (const float (*)[3])anchor_pos;
}

const coop_ekf_t *ekf_get_state(void)
{
    return &ekf;
}

bool ekf_get_peer_pos(uint16_t peer_id, float pos_out[3])
{
    /* In Phase 2, anchors are not in the state vector —
     * return the frozen position from anchor_pos[].                       */
    if (ekf_phase == EKF_PHASE_TAG_LOCALIZE) {
        int ai = anchor_index_of(peer_id);
        if (ai >= 0) {
            pos_out[0] = anchor_pos[ai][0];
            pos_out[1] = anchor_pos[ai][1];
            pos_out[2] = anchor_pos[ai][2];
            return true;
        }
    }

    /* Tags (Phase 2) and anchors (Phase 1) — look up by slot */
    int8_t slot = find_ekf_slot(peer_id);
    if (slot < 0) return false;
    pos_out[0] = ekf.x[slot*3 + 0];
    pos_out[1] = ekf.x[slot*3 + 1];
    pos_out[2] = ekf.x[slot*3 + 2];
    return true;
}