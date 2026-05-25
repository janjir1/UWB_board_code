#include "ekf.h"
#include "main.h"          /* LED_R_GPIO_Port, LED_R_Pin */
#include "../UWB_app/uwb_network.h"
#include "../Calculations/distance.h"
#include <math.h>
#include <string.h>
#include "../Generic/my_print.h"

/* HAL tick — avoids pulling in the full HAL chain. */
extern uint32_t HAL_GetTick(void);

/* ============================================================================
 * INITIAL POSITION ESTIMATES
 * -----------------------------------------------------------------------------
 * Rough starting positions for each known node, in metres relative to
 * Anchor 0 (world origin). A few metres of error is fine — the EKF will
 * converge to the correct position. The important thing is that the signs
 * are correct so the filter starts on the right side of the origin.
 *
 * Nodes not listed here get a small fixed jitter (±0.5 m) instead of a
 * random position, which also avoids the degenerate all-zeros start.
 *
 * Format: { node_id, x_m, y_m, z_m }
 * ============================================================================ */

/** @brief Spread of the Xorshift32 random seed used for unknown tag nodes (metres). */
#define EKF_INIT_RAND_M  2.0f

/* ============================================================================
 * HARDCODED Z HEIGHTS
 * -----------------------------------------------------------------------------
 * Anchor Z is supplied via the hint table (hint->z is used directly).
 * Tag Z is pinned to this constant for the duration of the experiment.
 * ============================================================================ */

/** @brief Tag height above the floor in metres. */
#define EKF_TAG_HARDCODED_Z  -0.589f

static const ekf_node_hint_t *g_ekf_pos_hints   = NULL;
static uint8_t                g_ekf_pos_hints_n  = 0;

/* ============================================================================
 * ANCHOR TABLE
 * -----------------------------------------------------------------------------
 * Edit the four network IDs below to match your hardware.
 * Index 0 is the world-origin anchor — always pinned at (0, 0, 0).
 * Indices 1–3 are surveyed in Phase 1; their positions are unknown at build time.
 *
 * Physical placement recommendation for good geometry (low DOP):
 *   Anchor 0 — one corner of the room (origin)
 *   Anchor 1 — opposite corner or at least >2 m away on any axis
 *   Anchor 2 — off the line formed by 0–1 (different Y or Z)
 *   Anchor 3 — elevated or offset so all four are non-coplanar
 * ============================================================================ */
const uint16_t EKF_ANCHOR_IDS[EKF_NUM_ANCHORS] = {
    0x63D8u,   /* ANCHOR 0 — origin (0, 0, 0) */
    0x91EDu,   /* ANCHOR 1 */
    0xC019u,   /* ANCHOR 2 */
    0x28CCu,   /* ANCHOR 3 */
};

/* ============================================================================
 * Internal state
 * ============================================================================ */

static coop_ekf_t  ekf;
static ekf_phase_t ekf_phase;

/** @brief Anchor positions frozen at end of Phase 1.
 *  anchor_pos[0] is always (0,0,0); indices 1–3 filled on Phase 1 convergence. */
static float anchor_pos[EKF_NUM_ANCHORS][3];

/** @brief Phase 1 step counter (gate for @c EKF_ANCHOR_MIN_STEPS). */
static uint32_t phase1_steps;

/** @brief World-origin constant — used throughout to avoid stack allocation. */
static const float k_origin[3] = { 0.0f, 0.0f, 0.0f };

/* -- Per-slot motion prior ------------------------------------------------ */
static float slot_vel_ema_h[EKF_MAX_PEERS]; /**< Horizontal EMA speed (m/s). */
static float slot_vel_ema_z[EKF_MAX_PEERS]; /**< Vertical EMA speed (m/s).   */
static float slot_last_x   [EKF_MAX_PEERS];
static float slot_last_y   [EKF_MAX_PEERS];
static float slot_hdg_x    [EKF_MAX_PEERS]; /**< Heading unit-vector x.       */
static float slot_hdg_y    [EKF_MAX_PEERS]; /**< Heading unit-vector y.        */
static bool  slot_hdg_valid[EKF_MAX_PEERS];

/* ============================================================================
 * Anchor helpers
 * ============================================================================ */

/**
 * @brief Return true when @p id matches one of the hardcoded anchor IDs.
 *
 * @param id  Node ID to test.
 * @return true if @p id is an anchor, false otherwise.
 */
static bool is_anchor_id(uint16_t id)
{
    for (uint8_t a = 0; a < EKF_NUM_ANCHORS; a++)
        if (EKF_ANCHOR_IDS[a] == id) return true;
    return false;
}

/**
 * @brief Return the anchor array index for @p id, or -1 if not an anchor.
 *
 * @param id  Node ID to look up.
 * @return Index in @c EKF_ANCHOR_IDS [0..EKF_NUM_ANCHORS-1], or -1.
 */
static int anchor_index_of(uint16_t id)
{
    for (int a = 0; a < (int)EKF_NUM_ANCHORS; a++)
        if (EKF_ANCHOR_IDS[a] == id) return a;
    return -1;
}

/**
 * @brief Map an anchor array index to its EKF slot index.
 *
 * Anchor 0 (origin) is a fixed reference with no slot — returns -1.
 * Self anchor returns @c ekf.self_slot.
 * Peer anchor searches @c peer_ids[]; returns -1 if not yet allocated.
 *
 * @param anchor_idx  Index into @c EKF_ANCHOR_IDS.
 * @return EKF slot index, or -1 if the anchor has no slot.
 */
static int anchor_idx_to_slot(int anchor_idx)
{
    if (anchor_idx == 0) return -1;

    uint16_t own_id = network_get_ownid();
    if (EKF_ANCHOR_IDS[anchor_idx] == own_id)
        return (int)ekf.self_slot;

    for (int p = 0; p < (int)ekf.n_peers; p++)
        if (ekf.peer_ids[p] == EKF_ANCHOR_IDS[anchor_idx]) return p;

    return -1;
}

/* ============================================================================
 * Low-level utilities
 * ============================================================================ */

/**
 * @brief Convert a scaled network distance value to metres.
 *
 * Sentinel values @c 0xFFFF and @c 0x0000 are treated as invalid.
 * EKF arithmetic is kept in single precision; @c dist_scale_to_ticks()
 * may use double internally, but the result is cast before use.
 *
 * @param dist_scaled  Packed distance value from the network peer state.
 * @return Distance in metres, or -1.0f if the value is invalid.
 */
static float range_scaled_to_m(uint16_t dist_scaled)
{
    if ((dist_scaled == 0xFFFFU) || (dist_scaled == 0x0000U))
        return -1.0f;

    float ticks = (float)dist_scale_to_ticks(dist_scaled);
    if (ticks <= 0.0f)
        return -1.0f;

    return ticks * (float)METERS_PER_TICK;
}

/**
 * @brief Convert a certainty byte to a range measurement standard deviation.
 *
 * Maps @p certainty linearly from [0, 255] → [EKF_SIGMA_MAX, EKF_SIGMA_MIN].
 *
 * @param certainty  Certainty value in [0, 255].
 * @return Corresponding measurement noise sigma in metres.
 */
float ekf_certainty_to_sigma(uint8_t certainty)
{
    float q = (float)certainty / 255.0f;
    return EKF_SIGMA_MIN + (EKF_SIGMA_MAX - EKF_SIGMA_MIN) * (1.0f - q);
}

/**
 * @brief Find the EKF slot index for a given peer ID.
 *
 * @param peer_id  Node ID to search for.
 * @return Slot index in [0, ekf.n_peers), or -1 if not found.
 */
static int8_t find_ekf_slot(uint16_t peer_id)
{
    for (int8_t p = 0; p < (int8_t)ekf.n_peers; p++) {
        if (ekf.peer_ids[p] == peer_id)
            return p;
    }
    return -1;
}

/**
 * @brief Return true if @p id is the local node or an active network peer.
 *
 * @param id  Node ID to check.
 * @return true if @p id is known, false otherwise.
 */
static bool network_contains_id(uint16_t id)
{
    if (id == network_get_ownid())
        return true;

    network_t *net = network_get_network();
    if (net == NULL)
        return false;

    for (int k = 0; k < (int)net->count; k++) {
        if (net->peers[k].id == id)
            return true;
    }
    return false;
}

/**
 * @brief Retrieve the peer state for a pair, trying both orderings.
 *
 * Checks (id_a → id_b) first; if NULL, falls back to (id_b → id_a).
 *
 * @param id_a  First node ID.
 * @param id_b  Second node ID.
 * @return Pointer to the peer state, or NULL if neither direction exists.
 */
static node_peer_state_t *get_peer_state_symmetric(uint16_t id_a, uint16_t id_b)
{
    node_peer_state_t *ps = network_get_peer_state(id_a, id_b);
    if (ps == NULL)
        ps = network_get_peer_state(id_b, id_a);
    return ps;
}

/**
 * @brief Write a position into the network self entry and any duplicate peer row.
 *
 * Some SYNC layouts keep the device's own ID inside @c net->peers[] as well as
 * @c net->self. Both rows are kept coherent so modules that read @c net->peers[]
 * directly receive consistent values. EKF logic always addresses self via
 * @c network_get_ownid() / @c self_slot and never allocates a duplicate slot.
 *
 * @param pos  3-element position array [x, y, z] in metres.
 */
static void publish_self_position(const float pos[3])
{
    float pos_tmp[3] = { pos[0], pos[1], pos[2] };
    mprintf("[EKF] publish_self pos=(%.3f, %.3f, %.3f)\n",
            (double)pos[0], (double)pos[1], (double)pos[2]);
    network_set_self_pos(pos_tmp);

    network_t *net    = network_get_network();
    uint16_t   own_id = network_get_ownid();
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

/**
 * @brief Reset the covariance block for one slot to @c EKF_INIT_P_POS.
 *
 * Zeroes all off-diagonal cross-terms for the slot and sets the diagonal
 * to @c EKF_INIT_P_POS. The Z diagonal entry is pinned to 0 since Z is
 * hardcoded and carries no uncertainty.
 *
 * @param slot     EKF slot index.
 * @param nActive  Active state dimension (= @c ekf.n_peers × 3).
 */
static void resetSlotCovariance(int slot, int nActive)
{
    int base = slot * 3;
    for (int k = 0; k < 3; k++) {
        int row = base + k;
        for (int j = 0; j < nActive; j++) {
            ekf.P[row][j] = 0.0f;
            ekf.P[j][row] = 0.0f;
        }
        ekf.P[row][row] = EKF_INIT_P_POS;
    }
    ekf.P[base + 2][base + 2] = 0.0f;
}

/**
 * @brief Reset per-slot motion state (EMA speeds, heading, stationary counter).
 *
 * @param p  EKF slot index.
 */
static void reset_slot_motion(int p)
{
    slot_vel_ema_h[p]        = 0.0f;
    slot_vel_ema_z[p]        = 0.0f;
    slot_hdg_valid[p]        = false;
    ekf.stationary_count[p]  = 0;
}

/**
 * @brief Allocate and initialise a new EKF slot.
 *
 * Initialises meta-fields (@c peer_ids, @c peer_away, @c peer_seeded,
 * @c peer_imu_valid) and sets the initial position from one of two paths:
 * - **Hint found**: position taken from @c g_ekf_pos_hints[]; slot marked
 *   seeded immediately so @ref seed_slot_centered does not overwrite it.
 * - **No hint**: position randomised with Xorshift32 in XY and pinned to
 *   @c EKF_TAG_HARDCODED_Z in Z.
 *
 * @param slot     Slot index to allocate.
 * @param id       Node ID to assign to the slot.
 * @param n_after  Active state dimension after this allocation (= @c n_peers × 3).
 */
static void alloc_slot(int slot, uint16_t id, int n_after)
{
    ekf.peer_ids[slot]       = id;
    ekf.peer_away[slot]      = false;
    ekf.peer_seeded[slot]    = false;
    ekf.peer_imu_valid[slot] = false;

    const ekf_node_hint_t *hint = NULL;
    for (uint8_t h = 0; h < g_ekf_pos_hints_n; h++) {
        if (g_ekf_pos_hints[h].id == id) {
            hint = &g_ekf_pos_hints[h];
            break;
        }
    }

    if (hint != NULL) {
        /* Known anchor — place at hint and mark seeded immediately.
         * Prevents seed_slot_centered() from overwriting the hint with
         * a random sphere placement when the first range arrives. */
        ekf.x[slot*3 + 0]       = hint->x;
        ekf.x[slot*3 + 1]       = hint->y;
        ekf.x[slot*3 + 2]       = hint->z;
        ekf.peer_seeded[slot]    = true;
        ekf.peer_imu_valid[slot] = true;
        slot_last_x[slot]        = hint->x;
        slot_last_y[slot]        = hint->y;
    } else {
        /* Unknown tag — random position inside the convex hull of anchors.
         * Spread is EKF_INIT_RAND_M in XY; Z is hardcoded. */
        uint32_t rng = (uint32_t)(HAL_GetTick()
                     ^ (uint32_t)id
                     ^ ((uint32_t)(unsigned)slot * 2654435761UL));
        rng ^= rng << 13; rng ^= rng >> 17; rng ^= rng << 5;
        ekf.x[slot*3 + 0] = ((float)(rng & 0xFFFFu) / 32767.5f - 1.0f) * EKF_INIT_RAND_M;
        rng ^= rng << 13; rng ^= rng >> 17; rng ^= rng << 5;
        ekf.x[slot*3 + 1] = ((float)(rng & 0xFFFFu) / 32767.5f - 1.0f) * EKF_INIT_RAND_M;
        ekf.x[slot*3 + 2] = EKF_TAG_HARDCODED_Z;
    }

    mprintf("[EKF] alloc slot=%d id=0x%04X %s pos=(%.2f, %.2f, %.2f)\n",
            slot, (unsigned)id,
            ekf.peer_seeded[slot] ? "hint" : "rng ",
            (double)ekf.x[slot*3+0], (double)ekf.x[slot*3+1], (double)ekf.x[slot*3+2]);

    ekf.Q[slot*3 + 2][slot*3 + 2] = 0.0f;
    reset_slot_motion(slot);
    resetSlotCovariance(slot, n_after);
}

/**
 * @brief Restore a previously-away peer when it returns to the network.
 *
 * Preserves the last known position (@c x[]) and @c peer_seeded flag so
 * the existing estimate is reused as the starting point. Inflates @c P so
 * incoming measurements are trusted and converge quickly.
 *
 * @param p  EKF slot index of the returning peer.
 */
static void slot_reconnect(int p)
{
    mprintf("[EKF] reconnect slot=%d id=0x%04X last_pos=(%.2f, %.2f, %.2f)\n",
            p, (unsigned)ekf.peer_ids[p],
            (double)ekf.x[p*3+0], (double)ekf.x[p*3+1], (double)ekf.x[p*3+2]);
    ekf.peer_away[p]      = false;
    ekf.peer_imu_valid[p] = false;
    reset_slot_motion(p);
    resetSlotCovariance(p, (int)ekf.n_peers * 3);
    /* peer_seeded[p] and x[p*3..] intentionally preserved. */
}

/**
 * @brief Reset all per-slot motion arrays to their zero/default state.
 */
static void reset_motion_arrays(void)
{
    for (int p = 0; p < EKF_MAX_PEERS; p++) {
        slot_vel_ema_h[p] = 0.0f;
        slot_vel_ema_z[p] = 0.0f;
        slot_last_x[p]    = 0.0f;
        slot_last_y[p]    = 0.0f;
        slot_hdg_x[p]     = 1.0f; /* unit-x default */
        slot_hdg_y[p]     = 0.0f;
        slot_hdg_valid[p] = false;
    }
}

/* ============================================================================
 * Seeding
 * ============================================================================ */

/**
 * @brief Place slot @p p at a random point on a sphere of radius @p range_m.
 *
 * The centre of the sphere is (@p cx, @p cy, @p cz). Azimuth is uniformly
 * distributed over [0, 2π); elevation fraction is uniformly distributed
 * in [0, 0.5] to avoid always seeding at Z = 0. Z is then overridden by
 * @c EKF_TAG_HARDCODED_Z regardless of the sphere geometry.
 *
 * @param p        EKF slot index to seed.
 * @param cx       Sphere centre X (m).
 * @param cy       Sphere centre Y (m).
 * @param cz       Sphere centre Z (m, unused after Z override).
 * @param range_m  Sphere radius in metres.
 */
static void seed_slot_centered(int p,
                                float cx, float cy, float cz,
                                float range_m)
{
    (void)cz; /* Z is overridden by EKF_TAG_HARDCODED_Z */

    uint32_t rng = (uint32_t)(HAL_GetTick()
                 ^ (uint32_t)ekf.peer_ids[p]
                 ^ ((uint32_t)(unsigned)p * 2654435761UL));
    rng ^= rng << 13; rng ^= rng >> 17; rng ^= rng << 5;

    float seed_angle = (float)rng * 1.46291808e-9f; /* [0, 2π) */

    uint32_t rng2 = rng ^ 0xDEADBEEFUL;
    rng2 ^= rng2 << 13; rng2 ^= rng2 >> 17; rng2 ^= rng2 << 5;
    float elev_frac = (float)(rng2 & 0xFFu) / 255.0f;
    float z_frac    = elev_frac * 0.5f;
    float xy_r      = sqrtf(fmaxf(0.0f,
                         range_m * range_m
                         - (range_m * z_frac) * (range_m * z_frac)));

    ekf.x[p*3 + 0]      = cx + xy_r * cosf(seed_angle);
    ekf.x[p*3 + 1]      = cy + xy_r * sinf(seed_angle);
    ekf.x[p*3 + 2]      = EKF_TAG_HARDCODED_Z;
    ekf.peer_seeded[p]   = true;
    ekf.peer_imu_valid[p] = true;
    slot_last_x[p]       = ekf.x[p*3 + 0];
    slot_last_y[p]       = ekf.x[p*3 + 1];
    slot_hdg_valid[p]    = false;

    mprintf("[EKF] seeded slot=%d id=0x%04X pos=(%.2f, %.2f, %.2f) r=%.2f\n",
            p, (unsigned)ekf.peer_ids[p],
            (double)ekf.x[p*3+0], (double)ekf.x[p*3+1], (double)ekf.x[p*3+2],
            (double)range_m);
}

/* ============================================================================
 * Sequential EKF range update
 * -----------------------------------------------------------------------------
 * apply_range_update_ext — scalar range measurement between two endpoints.
 *
 * fixed_i : if non-NULL, endpoint i is at this fixed position (not in state).
 * base_i  : state base index for i; ignored when fixed_i != NULL.
 * fixed_j : if non-NULL, endpoint j is at this fixed position.
 * base_j  : state base index for j; ignored when fixed_j != NULL.
 * range_m : measured distance (m).
 * sigma_d : measurement noise std-dev (m).
 * n       : active state dimension.
 *
 * Covers all cases in one function:
 *   fixed anchor → estimated tag : fixed_i = anchor_pos[a], fixed_j = NULL
 *   estimated   → estimated      : fixed_i = NULL,          fixed_j = NULL
 *   origin      → estimated      : fixed_i = k_origin,      fixed_j = NULL
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

    /* PH = P · H (sparse — only non-zero H entries contribute) */
    float PH[EKF_MAX_STATE];
    for (int i = 0; i < n; i++) {
        float acc = 0.0f;
        for (int j = 0; j < n; j++) acc += ekf.P[i][j] * H[j];
        PH[i] = acc;
    }

    /* Innovation variance S = H·P·H^T + R */
    float S = sigma_d * sigma_d;
    for (int j = 0; j < n; j++) S += H[j] * PH[j];

    /* Outlier gate */
    float innov = range_m - h;
    if (innov >  EKF_OUTLIER_GATE_POS * sqrtf(S)) return;
    if (innov < -EKF_OUTLIER_GATE_NEG * sqrtf(S)) return;

    /* Kalman gain K = PH / S */
    float S_inv = 1.0f / S;
    float K[EKF_MAX_STATE];
    for (int i = 0; i < n; i++) K[i] = PH[i] * S_inv;

    /* State update x += K · innov */
    for (int i = 0; i < n; i++) ekf.x[i] += K[i] * innov;

    /* Per-slot displacement clamp — limits how far a single update can
     * move any node. Protects against outliers that slip through the gate
     * during large initial uncertainty. */
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

    /* Joseph-form covariance update: P = P - K·PH^T - PH·K^T + S·K·K^T
     * Preserves symmetry exactly under finite-precision arithmetic. */
    for (int i = 0; i < n; i++) {
        for (int j = i; j < n; j++) {
            float v = ekf.P[i][j]
                    - K[i]  * PH[j]
                    - PH[i] * K[j]
                    + S     * K[i] * K[j];
            ekf.P[i][j] = ekf.P[j][i] = v;
        }
    }
}

/* ============================================================================
 * predict_anchor_slot (Phase 1 only)
 * -----------------------------------------------------------------------------
 * Anchors are stationary: no position update to x[].
 * A tiny process noise is added to P to maintain numerical health; it is
 * negligible compared to the range-update information rate.
 * ============================================================================ */
static void predict_anchor_slot(int p, float dt_s)
{
    int   base = p * 3;
    float q    = EKF_Q_H_FLOOR_STATIONARY * (dt_s / EKF_DT_NOM_S);
    for (int k = 0; k < 2; k++) {
        ekf.P[base+k][base+k] += q;
        if (ekf.P[base+k][base+k] > EKF_P_MAX)
            ekf.P[base+k][base+k] = EKF_P_MAX;
    }
}

/* ============================================================================
 * predict_slot (Phase 2)
 * ============================================================================ */

/**
 * @brief IMU-driven motion prediction for one tag slot.
 *
 * Tags are always treated as potentially moving. The IMU deadband suppresses
 * sensor noise floor; the EMA smooths the speed estimate. Z is pinned to
 * @c EKF_TAG_HARDCODED_Z to suppress vertical drift. A max-displacement clamp
 * prevents runaway prediction between measurement updates.
 *
 * Process noise is injected into @c P proportional to the smoothed speed.
 * A Cauchy-Schwarz clamp keeps the 3×3 self-block PSD after each step.
 *
 * @param p       EKF slot index.
 * @param vh_raw  Raw horizontal speed from IMU (m/s).
 * @param vz_raw  Raw vertical speed from IMU (m/s).
 * @param dt_s    Time step in seconds.
 */
static void predict_slot(int p, float vh_raw, float vz_raw, float dt_s)
{
    int   base  = p * 3;
    float pre_x = ekf.x[base + 0];
    float pre_y = ekf.x[base + 1];
    float pre_z = ekf.x[base + 2];

    float vh_eff = (fabsf(vh_raw) < EKF_IMU_DEADBAND_MS) ? 0.0f : vh_raw;
    float vz_eff = (fabsf(vz_raw) < EKF_IMU_DEADBAND_MS) ? 0.0f : vz_raw;

    slot_vel_ema_h[p] = EKF_VEL_EMA_ALPHA * vh_eff
                      + (1.0f - EKF_VEL_EMA_ALPHA) * slot_vel_ema_h[p];
    slot_vel_ema_z[p] = EKF_VEL_EMA_ALPHA * vz_eff
                      + (1.0f - EKF_VEL_EMA_ALPHA) * slot_vel_ema_z[p];

    ekf.x[base + 2] = EKF_TAG_HARDCODED_Z;

    if (slot_hdg_valid[p] && slot_vel_ema_h[p] > EKF_IMU_DEADBAND_MS) {
        ekf.x[base + 0] += slot_hdg_x[p] * slot_vel_ema_h[p] * dt_s;
        ekf.x[base + 1] += slot_hdg_y[p] * slot_vel_ema_h[p] * dt_s;
    }

    /* Max-displacement clamp */
    float ddx  = ekf.x[base+0] - pre_x;
    float ddy  = ekf.x[base+1] - pre_y;
    float dist = sqrtf(ddx*ddx + ddy*ddy);
    if (dist > EKF_MAX_STEP_M) {
        float scale = EKF_MAX_STEP_M / dist;
        ekf.x[base+0] = pre_x + ddx * scale;
        ekf.x[base+1] = pre_y + ddy * scale;
        ekf.x[base+2] = pre_z; /* restore clamped Z */
    }

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

/**
 * @brief Write the EKF estimate for slot @p p back into the network peer table.
 *
 * Skipped if @p net is NULL or the slot has not yet been seeded.
 *
 * @param p    EKF slot index.
 * @param net  Pointer to the active network state.
 */
static void write_slot_position_to_network(int p, network_t *net)
{
    if ((net == NULL) || (!ekf.peer_seeded[p]))
        return;

    for (int k = 0; k < (int)net->count; k++) {
        if (net->peers[k].id == ekf.peer_ids[p]) {
            net->peers[k].pos[0] = ekf.x[p*3 + 0];
            net->peers[k].pos[1] = ekf.x[p*3 + 1];
            net->peers[k].pos[2] = ekf.x[p*3 + 2];
            break;
        }
    }
}

/**
 * @brief Update the stored heading from position delta, then write back to network.
 *
 * Updates @c slot_hdg_x/y when the inter-step displacement exceeds
 * @c EKF_HDG_MIN_DELTA_M. Always updates @c slot_last_x/y and calls
 * @ref write_slot_position_to_network.
 *
 * @param p    EKF slot index.
 * @param net  Pointer to the active network state.
 */
static void update_heading_and_writeback(int p, network_t *net)
{
    if (!ekf.peer_seeded[p]) return;

    float dx   = ekf.x[p*3 + 0] - slot_last_x[p];
    float dy   = ekf.x[p*3 + 1] - slot_last_y[p];
    float dmag = sqrtf(dx*dx + dy*dy);
    if (dmag >= EKF_HDG_MIN_DELTA_M) {
        slot_hdg_x[p]    = dx / dmag;
        slot_hdg_y[p]    = dy / dmag;
        slot_hdg_valid[p] = true;
    }
    slot_last_x[p] = ekf.x[p*3 + 0];
    slot_last_y[p] = ekf.x[p*3 + 1];

    write_slot_position_to_network(p, net);
}

/* ============================================================================
 * ╔══════════════════════════════════════════════════════════════════════════╗
 * ║  PHASE 1 — ANCHOR SURVEY                                                ║
 * ╚══════════════════════════════════════════════════════════════════════════╝
 * ============================================================================ */

/**
 * @brief Reconcile the EKF slot table with the network for Phase 1.
 *
 * Only non-origin anchor IDs are given (or retain) slots.
 * Non-anchor (tag) peers are ignored entirely.
 *
 * - **Disconnect**: mark slot away, reset P (inflate), keep position.
 * - **Reconnect**: clear away flag, inflate P, keep position (@ref slot_reconnect).
 * - **New anchor**: allocate slot if not yet seen and present in the network.
 */
static void sync_peers_phase1(void)
{
    network_t *net    = network_get_network();
    uint16_t   own_id = network_get_ownid();
    int        n      = (int)ekf.n_peers * 3;

    /* Rules 1 & 2 — existing anchor slots */
    for (int p = 0; p < (int)ekf.n_peers; p++) {
        if (p == ekf.self_slot) continue;

        bool in_net = false;
        for (int k = 0; k < (int)net->count; k++)
            if (net->peers[k].id == ekf.peer_ids[p]) { in_net = true; break; }

        if (!in_net) {
            if (!ekf.peer_away[p]) {
                ekf.peer_away[p] = true;
                reset_slot_motion(p);
                resetSlotCovariance(p, n);
            }
        } else if (ekf.peer_away[p]) {
            slot_reconnect(p);
        }
    }

    /* Rule 3 — allocate slots for newly-seen non-origin anchors */
    for (int a = 1; a < (int)EKF_NUM_ANCHORS; a++) {
        uint16_t aid = EKF_ANCHOR_IDS[a];
        if (aid == own_id)          continue;
        if (find_ekf_slot(aid) >= 0) continue;

        bool in_net = false;
        for (int k = 0; k < (int)net->count; k++)
            if (net->peers[k].id == aid) { in_net = true; break; }
        if (!in_net)                    continue;
        if (ekf.n_peers >= EKF_MAX_PEERS) continue;

        int slot = (int)ekf.n_peers++;
        alloc_slot(slot, aid, (int)ekf.n_peers * 3);
    }
}

/**
 * @brief Return true when all four anchor IDs have been seen and are not away.
 *
 * Anchor 0 has no EKF slot and is checked directly via @ref network_contains_id.
 *
 * @return true if all anchors are present, false otherwise.
 */
static bool phase1_all_anchors_present(void)
{
    uint16_t own_id = network_get_ownid();

    for (int a = 0; a < (int)EKF_NUM_ANCHORS; a++) {
        if (EKF_ANCHOR_IDS[a] == own_id)
            continue;

        if (a == 0) {
            if (!network_contains_id(EKF_ANCHOR_IDS[a]))
                return false;
            continue;
        }

        int8_t slot = find_ekf_slot(EKF_ANCHOR_IDS[a]);
        if (slot < 0)              return false;
        if (ekf.peer_away[slot])   return false;
    }
    return true;
}

/**
 * @brief Return true when Phase 1 anchor survey has converged.
 *
 * Convergence requires:
 * -# At least @c EKF_ANCHOR_MIN_STEPS have been executed.
 * -# All four anchors are currently present.
 * -# Every estimated anchor slot is seeded and has @c P[k][k] < @c EKF_ANCHOR_P_CONVERGED.
 *
 * @return true if all convergence criteria are met, false otherwise.
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
 * ║  PHASE 1 → PHASE 2 TRANSITION                                           ║
 * ╚══════════════════════════════════════════════════════════════════════════╝
 * ============================================================================ */

static void ekf_init_phase2(void); /* forward declaration */

/**
 * @brief Freeze anchor positions and transition the EKF to Phase 2.
 *
 * Steps performed:
 * -# Anchor 0 is frozen at the world origin (0, 0, 0).
 * -# Estimated positions for anchors 1–3 are copied from the Phase 1 state.
 * -# LED_R is toggled once to signal successful anchor survey.
 * -# The EKF is re-initialised for Phase 2 tag tracking via @ref ekf_init_phase2.
 */
static void transition_to_phase2(void)
{
    uint16_t own_id = network_get_ownid();

    /* 1. Freeze anchor 0 at world origin. */
    anchor_pos[0][0] = 0.0f;
    anchor_pos[0][1] = 0.0f;
    anchor_pos[0][2] = 0.0f;

    /* 2. Copy estimated positions for anchors 1–3. */
    for (int p = 0; p < (int)ekf.n_peers; p++) {
        uint16_t pid = (p == ekf.self_slot) ? own_id : ekf.peer_ids[p];
        int      ai  = anchor_index_of(pid);
        if (ai <= 0) continue;

        anchor_pos[ai][0] = ekf.x[p*3 + 0];
        anchor_pos[ai][1] = ekf.x[p*3 + 1];
        anchor_pos[ai][2] = ekf.x[p*3 + 2];
    }

    /* 3. Signal success. */
    mprintf("[EKF] phase1->phase2 after %lu steps anchors: "
            "(%.2f,%.2f,%.2f) (%.2f,%.2f,%.2f) "
            "(%.2f,%.2f,%.2f) (%.2f,%.2f,%.2f)\n",
            (unsigned long)phase1_steps,
            (double)anchor_pos[0][0], (double)anchor_pos[0][1], (double)anchor_pos[0][2],
            (double)anchor_pos[1][0], (double)anchor_pos[1][1], (double)anchor_pos[1][2],
            (double)anchor_pos[2][0], (double)anchor_pos[2][1], (double)anchor_pos[2][2],
            (double)anchor_pos[3][0], (double)anchor_pos[3][1], (double)anchor_pos[3][2]);
    HAL_GPIO_TogglePin(LED_R_GPIO_Port, LED_R_Pin);
    ekf_phase = EKF_PHASE_TAG_LOCALIZE;

    /* 4. Re-initialise filter for Phase 2. */
    ekf_init_phase2();
}

/* ============================================================================
 * ekf_step_phase1
 * -----------------------------------------------------------------------------
 * One step of the Phase 1 anchor-survey EKF.
 *
 * PREDICT : tiny stationary noise only (no motion update).
 * UPDATE  : all available inter-anchor range pairs.
 *
 * Uses only the four hardcoded anchor IDs as measurement sources.
 * Non-anchor nodes in the network are ignored.
 * ============================================================================ */
static void ekf_step_phase1(float dt_s)
{
    sync_peers_phase1();
    int n = (int)ekf.n_peers * 3;

    /* -- PREDICT ----------------------------------------------------------- */
    for (int p = 0; p < (int)ekf.n_peers; p++) {
        if (ekf.peer_away[p]) continue;
        predict_anchor_slot(p, dt_s);
    }

    /* -- UPDATE — all anchor pairs (ai < aj) -------------------------------- */
    for (int ai = 0; ai < (int)EKF_NUM_ANCHORS; ai++) {
        for (int aj = ai + 1; aj < (int)EKF_NUM_ANCHORS; aj++) {

            int slot_i = anchor_idx_to_slot(ai);
            int slot_j = anchor_idx_to_slot(aj);

            if (ai > 0 && (slot_i < 0 || ekf.peer_away[slot_i])) continue;
            if (aj > 0 && (slot_j < 0 || ekf.peer_away[slot_j])) continue;

            node_peer_state_t *ps =
                get_peer_state_symmetric(EKF_ANCHOR_IDS[ai], EKF_ANCHOR_IDS[aj]);
            if (ps == NULL) continue;

            float range_m = range_scaled_to_m(ps->distance_scaled);
            if (range_m < 0.0f) continue;

            float sigma_d = ekf_certainty_to_sigma(ps->certainty);

            bool seeded_i = (ai == 0) || ekf.peer_seeded[slot_i];
            bool seeded_j = (aj == 0) || ekf.peer_seeded[slot_j];

            /* Seed from a known endpoint. If a slot was pre-seeded from
             * g_ekf_pos_hints[] in alloc_slot(), peer_seeded[] is already true —
             * do NOT call seed_slot_centered() and overwrite the hint. */
            if ((aj > 0) && (!seeded_j) && seeded_i) {
                if (ekf.peer_seeded[slot_j]) {
                    seeded_j = true;
                } else {
                    float pi_x = (ai == 0) ? 0.0f : ekf.x[slot_i*3 + 0];
                    float pi_y = (ai == 0) ? 0.0f : ekf.x[slot_i*3 + 1];
                    float pi_z = (ai == 0) ? 0.0f : ekf.x[slot_i*3 + 2];
                    seed_slot_centered(slot_j, pi_x, pi_y, pi_z, range_m);
                    seeded_j = true;
                }
            }

            if ((ai > 0) && (!seeded_i) && seeded_j) {
                if (ekf.peer_seeded[slot_i]) {
                    seeded_i = true;
                } else {
                    seed_slot_centered(slot_i,
                                       ekf.x[slot_j*3 + 0],
                                       ekf.x[slot_j*3 + 1],
                                       ekf.x[slot_j*3 + 2],
                                       range_m);
                    seeded_i = true;
                }
            }

            if ((!seeded_i) || (!seeded_j)) continue;

            const float *fp_i = (ai == 0) ? k_origin : NULL;
            const float *fp_j = (aj == 0) ? k_origin : NULL;
            apply_range_update_ext(fp_i, (ai == 0) ? 0 : slot_i * 3,
                                   fp_j, (aj == 0) ? 0 : slot_j * 3,
                                   range_m, sigma_d, n);
        }
    }

    phase1_steps++;
    ekf.initialised = true;

    /* -- WRITE-BACK: publish own position if self is an anchor ----------- */
    uint16_t own_id    = network_get_ownid();
    int      own_anch  = anchor_index_of(own_id);

    if (own_anch == 0) {
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

    /* -- CONVERGENCE CHECK ------------------------------------------------ */
    if (phase1_converged()) {
        mprintf("[EKF] phase1 converged after %lu steps\n",
                (unsigned long)phase1_steps);
        transition_to_phase2();
    }
}

/* ============================================================================
 * ╔══════════════════════════════════════════════════════════════════════════╗
 * ║  PHASE 2 — TAG LOCALIZATION                                              ║
 * ╚══════════════════════════════════════════════════════════════════════════╝
 * ============================================================================ */

/**
 * @brief Reconcile the EKF slot table with the network for Phase 2.
 *
 * Anchor IDs are skipped — they are fixed reference points, not EKF state slots.
 *
 * - **Away**: P inflated, last-known position preserved for stability.
 * - **Return**: @ref slot_reconnect — position preserved, P inflated, IMU reset.
 * - **New non-anchor peer**: slot allocated via @ref alloc_slot.
 */
static void sync_peers_phase2(void)
{
    network_t *net    = network_get_network();
    uint16_t   own_id = network_get_ownid();
    int        n      = (int)ekf.n_peers * 3;

    /* Rules 1 & 2 */
    for (int p = 0; p < (int)ekf.n_peers; p++) {
        if (p == ekf.self_slot)             continue;
        if (is_anchor_id(ekf.peer_ids[p])) continue;

        bool in_net = false;
        for (int k = 0; k < (int)net->count; k++)
            if (net->peers[k].id == ekf.peer_ids[p]) { in_net = true; break; }

        if (!in_net) {
            if (!ekf.peer_away[p]) {
                ekf.peer_away[p] = true;
                reset_slot_motion(p);
                resetSlotCovariance(p, n);
                /* x[p*3..] and peer_seeded[p] preserved — last known position kept. */
            }
        } else if (ekf.peer_away[p]) {
            slot_reconnect(p);
        }
    }

    /* Rule 3 — allocate slots for newly-seen non-anchor peers */
    for (int k = 0; k < (int)net->count; k++) {
        uint16_t pid = net->peers[k].id;
        if (pid == 0 || pid == own_id)  continue;
        if (is_anchor_id(pid))          continue;
        if (find_ekf_slot(pid) >= 0)    continue;
        if (ekf.n_peers >= EKF_MAX_PEERS) continue;

        int slot = (int)ekf.n_peers++;
        alloc_slot(slot, pid, (int)ekf.n_peers * 3);
    }
}

/**
 * @brief Initialise the EKF for Phase 2 tag tracking.
 *
 * Called from @ref transition_to_phase2. Clears the EKF struct and allocates
 * slots for all non-anchor devices currently visible in the network.
 * Self is given a slot only if it is a tag (not an anchor).
 */
static void ekf_init_phase2(void)
{
    mprintf("[EKF] ekf_init_phase2 called\n");
    memset(&ekf, 0, sizeof(ekf));
    reset_motion_arrays();

    ekf.self_slot    = -1;
    ekf.initialised  = false;
    ekf.last_tick_ms = HAL_GetTick();

    network_t *net    = network_get_network();
    uint16_t   own_id = network_get_ownid();

    /* Self slot — only allocated if self is a tag. */
    if (!is_anchor_id(own_id)) {
        int slot = (int)ekf.n_peers++;
        ekf.self_slot = (int8_t)slot;
        alloc_slot(slot, own_id, (int)ekf.n_peers * 3);
        ekf.peer_imu_valid[slot] = true;
    }

    /* Peer slots for all non-anchor devices currently in network. */
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
    for (int i = 0; i < n; i++) {
        if (i % 3 == 2) continue; /* Z pinned to 0 in resetSlotCovariance */
        ekf.P[i][i] = EKF_INIT_P_POS;
    }
    for (int p = 0; p < (int)ekf.n_peers; p++)
        ekf.Q[p*3+2][p*3+2] = 0.0f;
}

/**
 * @brief One step of the Phase 2 tag-localization EKF.
 *
 * **PREDICT**: IMU-driven motion prediction for each tag slot.
 *
 * **UPDATE 1 — anchor → tag**: applies a fixed-anchor range update for
 * every (anchor, tag) pair with a valid measurement. Seeds unseeded slots
 * from the first available anchor distance.
 *
 * **UPDATE 2 — tag → tag**: applies a floating range update for every
 * seeded, active tag pair.
 *
 * **WRITE-BACK**: publishes frozen anchor positions into @c net->peers[] for
 * external consumers; publishes estimated tag positions and self position.
 * Away-tag positions are preserved in the EKF state but not written into
 * @c net->peers[] (no active row for them).
 *
 * @param az_self_ms  Self vertical speed from IMU (m/s).
 * @param ah_self_ms  Self horizontal speed from IMU (m/s).
 * @param dt_s        Time step in seconds.
 */
static void ekf_step_phase2(float az_self_ms, float ah_self_ms, float dt_s)
{
    sync_peers_phase2();
    int      n      = (int)ekf.n_peers * 3;
    uint16_t own_id = network_get_ownid();

    /* -- PREDICT ----------------------------------------------------------- */
    for (int p = 0; p < (int)ekf.n_peers; p++) {
        if (ekf.peer_away[p]) continue;

        float vh_raw, vz_raw;
        if (p == ekf.self_slot) {
            vh_raw = ah_self_ms;
            vz_raw = az_self_ms;
        } else {
            vh_raw = 0.0f;
            vz_raw = 0.0f;

            node_t *peer_node = find_peer(ekf.peer_ids[p]);
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

    /* -- UPDATE 1: anchor → tag --------------------------------------------- */
    for (int p = 0; p < (int)ekf.n_peers; p++) {
        if (ekf.peer_away[p]) continue;
        uint16_t pid = (p == ekf.self_slot) ? own_id : ekf.peer_ids[p];

        for (int a = 0; a < (int)EKF_NUM_ANCHORS; a++) {
            node_peer_state_t *ps = get_peer_state_symmetric(EKF_ANCHOR_IDS[a], pid);
            if (ps == NULL) continue;

            float range_m = range_scaled_to_m(ps->distance_scaled);
            if (range_m < 0.0f) continue;

            if (!ekf.peer_seeded[p]) {
                mprintf("[EKF] phase2 seed slot=%d from anchor=%d range=%.2f\n",
                        p, a, (double)range_m);
                seed_slot_centered(p, anchor_pos[a][0], anchor_pos[a][1],
                                      anchor_pos[a][2], range_m);
            }

            float sigma_d = ekf_certainty_to_sigma(ps->certainty);
            apply_range_update_ext(anchor_pos[a], 0,
                                   NULL, p * 3,
                                   range_m, sigma_d, n);
        }
    }

    /* -- UPDATE 2: tag → tag ------------------------------------------------ */
    for (int pi = 0; pi < (int)ekf.n_peers; pi++) {
        if (ekf.peer_away[pi] || !ekf.peer_seeded[pi]) continue;

        for (int pj = pi + 1; pj < (int)ekf.n_peers; pj++) {
            if (ekf.peer_away[pj] || !ekf.peer_seeded[pj]) continue;

            uint16_t id_i = (pi == ekf.self_slot) ? own_id : ekf.peer_ids[pi];
            uint16_t id_j = (pj == ekf.self_slot) ? own_id : ekf.peer_ids[pj];
            node_peer_state_t *ps = get_peer_state_symmetric(id_i, id_j);
            if (ps == NULL) continue;

            float range_m = range_scaled_to_m(ps->distance_scaled);
            if (range_m < 0.0f) continue;

            float sigma_d = ekf_certainty_to_sigma(ps->certainty);
            apply_range_update_ext(NULL, pi * 3,
                                   NULL, pj * 3,
                                   range_m, sigma_d, n);
        }
    }

    ekf.initialised = true;

    /* -- WRITE-BACK -------------------------------------------------------- */
    network_t *net = network_get_network();

    /* Publish frozen anchor positions into net->peers[]. */
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

    /* Publish estimated tag positions for active peers.
     * Away-tag positions are preserved in EKF state but not written to
     * net->peers[] — there is no active row for them. */
    for (int p = 0; p < (int)ekf.n_peers; p++) {
        if (p == ekf.self_slot) {
            if (ekf.peer_seeded[p]) {
                float sp[3] = { ekf.x[p*3+0], ekf.x[p*3+1], ekf.x[p*3+2] };
                publish_self_position(sp);

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
        }
        /* Away slot: local EKF position preserved, not published. */
    }

    /* If self is an anchor in Phase 2, publish its frozen position. */
    int own_anch = anchor_index_of(own_id);
    if (own_anch >= 0)
        publish_self_position(anchor_pos[own_anch]);
}

/* ============================================================================
 * ╔══════════════════════════════════════════════════════════════════════════╗
 * ║  PUBLIC API                                                              ║
 * ╚══════════════════════════════════════════════════════════════════════════╝
 * ============================================================================ */

/**
 * @brief Initialise (or soft-reset) the cooperative EKF.
 *
 * If the EKF is already in Phase 2 (tag localization) — e.g. after a
 * self-disconnect/reconnect — only stale motion priors are reset and @c P
 * is inflated; the solved anchor map and last-known tag positions are
 * preserved so the filter recovers quickly without repeating Phase 1.
 *
 * On a fresh start (Phase 1) the full filter state is zeroed, slots are
 * allocated for all visible anchor peers, and initial covariances are set.
 *
 * @param pos_hints    Array of node ID / position hint pairs, or NULL.
 * @param pos_hints_n  Length of @p pos_hints.
 */
void ekf_init(const ekf_node_hint_t *pos_hints, uint8_t pos_hints_n)
{
    g_ekf_pos_hints   = pos_hints;
    g_ekf_pos_hints_n = pos_hints_n;

    if (ekf_phase == EKF_PHASE_TAG_LOCALIZE) {
        int      n      = (int)ekf.n_peers * 3;
        uint16_t own_id = network_get_ownid();

        reset_motion_arrays();
        for (int p = 0; p < (int)ekf.n_peers; p++) {
            reset_slot_motion(p);
            resetSlotCovariance(p, n);
            ekf.peer_imu_valid[p] = (ekf.peer_ids[p] == own_id);
        }
        ekf.last_tick_ms = HAL_GetTick();
        return;
    }

    memset(&ekf, 0, sizeof(ekf));
    memset(anchor_pos, 0, sizeof(anchor_pos));
    reset_motion_arrays();

    ekf_phase    = EKF_PHASE_ANCHOR_SURVEY;
    phase1_steps = 0;

    ekf.self_slot    = -1;
    ekf.initialised  = false;
    ekf.last_tick_ms = HAL_GetTick();

    network_t *net    = network_get_network();
    uint16_t   own_id = network_get_ownid();

    int self_anchor_idx = anchor_index_of(own_id);

    if (self_anchor_idx > 0) {
        /* Self is a non-origin anchor — allocate self_slot for Phase 1.
         * Anchors are stationary; imu_valid is false to prevent accidental
         * motion updates if predict_slot is ever called for this slot. */
        int slot = (int)ekf.n_peers++;
        ekf.self_slot = (int8_t)slot;
        alloc_slot(slot, own_id, (int)ekf.n_peers * 3);
        ekf.peer_imu_valid[slot] = false;
    }
    /* self_anchor_idx == 0 : self is the origin anchor, no slot needed. */
    /* self_anchor_idx == -1: self is a tag, no slot during Phase 1.     */

    /* Allocate slots for non-origin anchor peers currently in network. */
    for (int a = 1; a < (int)EKF_NUM_ANCHORS; a++) {
        uint16_t aid = EKF_ANCHOR_IDS[a];
        if (aid == own_id)           continue;
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
    for (int i = 0; i < n; i++) {
        if (i % 3 == 2) continue; /* Z pinned to 0 in resetSlotCovariance */
        ekf.P[i][i] = EKF_INIT_P_POS;
    }
    for (int p = 0; p < (int)ekf.n_peers; p++)
        ekf.Q[p*3+2][p*3+2] = 0.0f;
}

/**
 * @brief Advance the EKF by one step, dispatching to the current phase.
 *
 * Computes a variable @c dt from @c HAL_GetTick(). On the first call
 * (@c !ekf.initialised) @c EKF_DT_NOM_S is used. The step is clamped to
 * [@c 0.001 s, @c EKF_DT_MAX_S] to protect against missed ticks.
 *
 * @param az_self_ms  Self vertical speed from IMU (m/s).
 * @param ah_self_ms  Self horizontal speed from IMU (m/s).
 */
void ekf_step(float az_self_ms, float ah_self_ms)
{
    uint32_t now_ms = HAL_GetTick();
    float    dt_s;

    if (!ekf.initialised) {
        dt_s = EKF_DT_NOM_S;
    } else {
        dt_s = (float)((uint32_t)(now_ms - ekf.last_tick_ms)) * 0.001f;
        if (dt_s < 0.001f)      dt_s = EKF_DT_NOM_S;
        if (dt_s > EKF_DT_MAX_S) dt_s = EKF_DT_MAX_S;
    }
    ekf.last_tick_ms = now_ms;

    if (ekf_phase == EKF_PHASE_ANCHOR_SURVEY)
        ekf_step_phase1(dt_s);
    else
        ekf_step_phase2(az_self_ms, ah_self_ms, dt_s);
}

/**
 * @brief Return true if Phase 1 anchor survey has completed.
 *
 * @return true if the EKF is in @c EKF_PHASE_TAG_LOCALIZE, false otherwise.
 */
bool ekf_anchor_ready(void)
{
    return (ekf_phase == EKF_PHASE_TAG_LOCALIZE);
}

/**
 * @brief Return the current EKF phase.
 *
 * @return @c EKF_PHASE_ANCHOR_SURVEY or @c EKF_PHASE_TAG_LOCALIZE.
 */
ekf_phase_t ekf_get_phase(void)
{
    return ekf_phase;
}

/**
 * @brief Return a pointer to the frozen anchor position array.
 *
 * Valid after Phase 1 convergence. Anchor 0 is always (0,0,0).
 *
 * @return Pointer to @c anchor_pos[EKF_NUM_ANCHORS][3].
 */
const float (*ekf_get_anchor_positions(void))[3]
{
    return (const float (*)[3])anchor_pos;
}

/**
 * @brief Return a read-only pointer to the internal EKF state.
 *
 * @return Pointer to the @c coop_ekf_t struct.
 */
const coop_ekf_t *ekf_get_state(void)
{
    return &ekf;
}

/**
 * @brief Look up the current estimated position of a peer or anchor.
 *
 * In Phase 2 anchors are not in the state vector — the frozen position
 * from @c anchor_pos[] is returned directly. Tags (Phase 2) and anchors
 * (Phase 1) are found by slot.
 *
 * @param peer_id   Node ID to query.
 * @param pos_out   Output array [x, y, z] in metres.
 * @return true if the position was found, false if the ID is unknown.
 */
bool ekf_get_peer_pos(uint16_t peer_id, float pos_out[3])
{
    if (ekf_phase == EKF_PHASE_TAG_LOCALIZE) {
        int ai = anchor_index_of(peer_id);
        if (ai >= 0) {
            pos_out[0] = anchor_pos[ai][0];
            pos_out[1] = anchor_pos[ai][1];
            pos_out[2] = anchor_pos[ai][2];
            return true;
        }
    }

    int8_t slot = find_ekf_slot(peer_id);
    if (slot < 0) return false;
    pos_out[0] = ekf.x[slot*3 + 0];
    pos_out[1] = ekf.x[slot*3 + 1];
    pos_out[2] = ekf.x[slot*3 + 2];
    return true;
}