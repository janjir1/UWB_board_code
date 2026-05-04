/* ============================================================================
 * ekf.c — Cooperative UWB + IMU Joint Extended Kalman Filter
 *
 * Rewritten to fix all known bugs:
 *   Bug #2  — IMU zero-init decodes to -4 m/s: guarded by peer_imu_valid[].
 *   Bug #4  — P symmetry loss: Joseph-form update + upper-triangle mirror.
 *   Bug #9  — ~300 ppm scale error: uses METERS_PER_TICK from distance.h,
 *             not the hardcoded 299 792 458 m/s constant.
 *   Bug #10 — Q_h overestimate (sum instead of RMS): vh_rms²×dt² + floor.
 *   Bug #11 — Peers joining after init never seeded: per-peer peer_seeded[].
 *   Bug #12 — 0x0000 ingested as valid 0 m range: range_scaled_to_m() guards
 *             both 0x0000 (zero-init sentinel) and 0xFFFF (no-measurement).
 *   Bug #13 — Degenerate collinear seed: all peers placed at (range_m, 0, 0)
 *             kept y=0 permanently. Fixed with random-angle seeding on a
 *             circle of radius range_m (xorshift32 hash, no stdlib needed).
 *   Bug #14 — EKF_P_MAX diagonal-only cap can break positive semi-definiteness:
 *             off-diagonal terms uncapped → |P_ij| > sqrt(P_ii·P_jj) possible
 *             → S = H·P·H^T + R can go negative → sqrtf(S) = NaN. Fixed with
 *             Cauchy-Schwarz clamp on the 3×3 peer block after each predict.
 *
 * Additional improvements:
 *   - Variable dt from HAL_GetTick(), capped at EKF_DT_MAX_S.
 *   - Away-peer handling: sync_peers() marks absent peers peer_away=true,
 *     resets their covariance immediately (prevents ghost-correlation updates),
 *     and re-seeds returning peers from scratch.
 *   - P diagonal clamped to EKF_P_MAX after each predict step.
 *   - Outlier gate uses sqrt(S) where S = H*P*H^T + R (dynamic, not sigma_d).
 *   - ekf_get_peer_pos() returns last-known position for away peers too.
 *   - network_set_self_pos([0,0,0]) called unconditionally every step.
 *   - ekf_init() is never called for peer join/leave events.
 * ============================================================================ */

#include "ekf.h"
#include "../UWB_app/uwb_network.h"
#include "../Calculations/distance.h"
#include <math.h>
#include <string.h>

/* Avoid pulling in the full STM32 HAL header chain for one function. */
extern uint32_t HAL_GetTick(void);

/* ============================================================================
 * Internal state
 * ============================================================================ */
static coop_ekf_t ekf;

/* ============================================================================
 * range_scaled_to_m
 *   Convert a network-layer encoded distance to metres.
 *   Returns -1.0f on any invalid or sentinel value.
 *
 *   Bug #12: 0x0000 is the zero-init sentinel (peer slot never measured).
 *            Original code only guarded 0xFFFF; 0x0000 decoded as ~0 m.
 *   Bug #9:  Original code used UWB_TICKS_PER_METRE computed from c=299 792 458,
 *            which disagrees with METERS_PER_TICK in distance.h by ~300 ppm
 *            (distance.h uses SPEED_OF_LIGHT = 299 702 547.0, the DW3000
 *            empirically calibrated value).  Use (double)METERS_PER_TICK
 *            to avoid the float-precision loss on the cast while keeping the
 *            correct constant.
 * ============================================================================ */
static float range_scaled_to_m(uint16_t dist_scaled)
{
    if (dist_scaled == 0xFFFFU || dist_scaled == 0x0000U) return -1.0f;
    double ticks = dist_scale_to_ticks(dist_scaled);
    if (ticks <= 0.0) return -1.0f;
    return (float)(ticks * (double)METERS_PER_TICK);
}

/* ============================================================================
 * ekf_certainty_to_sigma
 * ============================================================================ */
float ekf_certainty_to_sigma(uint8_t certainty)
{
    float q = (float)certainty / 255.0f;
    return EKF_SIGMA_MIN + (EKF_SIGMA_MAX - EKF_SIGMA_MIN) * (1.0f - q);
}

/* ============================================================================
 * find_ekf_slot
 *   Map a network peer_id to its EKF slot index [0 .. n_peers-1].
 *   Returns -1 if not found.
 * ============================================================================ */
static int8_t find_ekf_slot(uint16_t peer_id)
{
    for (int8_t p = 0; p < (int8_t)ekf.n_peers; p++)
        if (ekf.peer_ids[p] == peer_id) return p;
    return -1;
}

/* ============================================================================
 * reset_peer_covariance
 *   Zero all cross-covariances coupling peer 'slot' to every other peer,
 *   then reset the peer's own P diagonal to EKF_INIT_P_POS.
 *
 *   Called in two situations:
 *     1. Peer goes away  — prevents K[away] ≠ 0 ghost updates from corrupting
 *        the away peer's state while present-peer measurements continue.
 *     2. Peer returns    — discards stale cross-covariances accumulated during
 *        absence; re-seeding will re-establish correct correlations.
 *
 *   n_active : current total state dimension  (ekf.n_peers * 3)
 * ============================================================================ */
static void reset_peer_covariance(int slot, int n_active)
{
    int base = slot * 3;

    for (int k = 0; k < 3; k++) {
        int row = base + k;
        /* Zero the entire row and symmetric column. */
        for (int j = 0; j < n_active; j++) {
            ekf.P[row][j] = 0.0f;
            ekf.P[j][row] = 0.0f;
        }
        /* Then set the diagonal element (was zeroed in the loop above). */
        ekf.P[row][row] = EKF_INIT_P_POS;
    }
}

/* ============================================================================
 * sync_peers
 *   Reconcile the EKF peer table with the current network_t state.
 *
 *   Rule 1 — Peer present in EKF, absent from network:
 *     Mark peer_away[p] = true.
 *     Reset covariance immediately so subsequent present-peer updates cannot
 *     bleed into the absent peer's state via non-zero cross-covariances.
 *
 *   Rule 2 — Peer present in EKF and back in network (was away):
 *     Clear peer_away[p], peer_seeded[p], peer_imu_valid[p].
 *     Zero state block and reset covariance — filter re-acquires from scratch.
 *
 *   Rule 3 — Peer in network, no EKF slot yet:
 *     Allocate new slot.  Zero state, zero cross-covariances, P diagonal →
 *     EKF_INIT_P_POS, Q[z] → EKF_Q_Z_POS.
 *
 *   Slots are NEVER freed (n_peers never decreases).  Away peers continue to
 *   occupy their index; ekf_get_peer_pos() can still return their last fix.
 *
 *   Never calls ekf_init().
 * ============================================================================ */
static void sync_peers(void)
{
    network_t *net    = network_get_network();
    uint16_t   own_id = network_get_ownid();
    int        n      = (int)ekf.n_peers * 3;   /* current state dim (before adds) */

    /* ---- Rule 1 & 2: walk existing EKF slots -------------------------------- */
    for (int p = 0; p < (int)ekf.n_peers; p++) {

        bool in_net = false;
        for (int k = 0; k < (int)net->count; k++) {
            if (net->peers[k].id == ekf.peer_ids[p]) { in_net = true; break; }
        }

        if (!in_net) {
            /* Peer has left. */
            if (!ekf.peer_away[p]) {
                ekf.peer_away[p] = true;
                reset_peer_covariance(p, n);   /* kill ghost-correlation risk */
            }
        } else {
            /* Peer is present. */
            if (ekf.peer_away[p]) {
                /* Peer has returned — re-acquire from scratch. */
                ekf.peer_away[p]      = false;
                ekf.peer_seeded[p]    = false;
                ekf.peer_imu_valid[p] = false;
                ekf.x[p*3+0] = 0.0f;
                ekf.x[p*3+1] = 0.0f;
                ekf.x[p*3+2] = 0.0f;
                reset_peer_covariance(p, n);
            }
        }
    }

    /* ---- Rule 3: add new peers --------------------------------------------- */
    for (int k = 0; k < (int)net->count; k++) {
        uint16_t pid = net->peers[k].id;
        if (pid == 0 || pid == own_id)  continue;  /* unused slot or self */
        if (find_ekf_slot(pid) >= 0)    continue;  /* already tracked     */
        if (ekf.n_peers >= EKF_MAX_PEERS) continue; /* table full          */

        int slot = (int)ekf.n_peers;
        ekf.n_peers++;
        int new_n = (int)ekf.n_peers * 3;

        ekf.peer_ids[slot]       = pid;
        ekf.peer_away[slot]      = false;
        ekf.peer_seeded[slot]    = false;
        ekf.peer_imu_valid[slot] = false;

        ekf.x[slot*3+0] = 0.0f;
        ekf.x[slot*3+1] = 0.0f;
        ekf.x[slot*3+2] = 0.0f;

        /* Q Z-entry initialised here; X/Y Q is computed dynamically in predict. */
        ekf.Q[slot*3+2][slot*3+2] = EKF_Q_Z_POS;

        /* Zero cross-covariances to existing peers; set diagonal. */
        reset_peer_covariance(slot, new_n);
    }
}

/* ============================================================================
 * apply_range_update
 *   Sequential EKF update for one scalar range measurement.
 *
 *   base_i, base_j : state base indices of the two endpoints.
 *                    These are ignored (not accessed) when the corresponding
 *                    self_x flag is true — pass 0 as a safe dummy.
 *   self_i, self_j : true when the endpoint is self, fixed at [0,0,0].
 *   range_m        : measured distance in metres (must be > 0).
 *   sigma_d        : measurement noise std-dev in metres.
 *   n              : active state dimension  (ekf.n_peers * 3).
 *
 *   Jacobian H (1×n, sparse):
 *     H[base_i .. base_i+2] = −unit(i→j)   when !self_i
 *     H[base_j .. base_j+2] = +unit(i→j)   when !self_j
 *     (derivative of ‖xⱼ − xᵢ‖ w.r.t. each position component)
 *
 *   Outlier gate (Bug fix):
 *     |innov| > EKF_OUTLIER_GATE × sqrt(S),  S = H*P*H^T + R
 *     (original code gated on sigma_d, which ignores P and can mis-reject
 *      valid measurements during filter convergence)
 *
 *   Covariance update — Joseph form (Bug #4):
 *     P_new[i][j] = P[i][j] − K[i]·PH[j] − PH[i]·K[j] + S·K[i]·K[j]
 *     where PH = P·H^T (precomputed before any write).
 *     Upper triangle computed, then mirrored to lower — guarantees symmetry.
 * ============================================================================ */
static void apply_range_update(int base_i, int base_j,
                               bool self_i, bool self_j,
                               float range_m, float sigma_d,
                               int n)
{
    /* Positions of both endpoints in the current state estimate. */
    float xi = self_i ? 0.0f : ekf.x[base_i + 0];
    float yi = self_i ? 0.0f : ekf.x[base_i + 1];
    float zi = self_i ? 0.0f : ekf.x[base_i + 2];
    float xj = self_j ? 0.0f : ekf.x[base_j + 0];
    float yj = self_j ? 0.0f : ekf.x[base_j + 1];
    float zj = self_j ? 0.0f : ekf.x[base_j + 2];

    /* Predicted range h = ‖xⱼ − xᵢ‖ */
    float dx = xj - xi;
    float dy = yj - yi;
    float dz = zj - zi;
    float h  = sqrtf(dx*dx + dy*dy + dz*dz);
    if (h < 1.0e-4f) h = 1.0e-4f;   /* guard: prevent divide-by-zero at origin */

    /* Sparse Jacobian H (1×n) */
    float H[EKF_MAX_STATE];
    memset(H, 0, sizeof(H));

    float ux = dx / h;
    float uy = dy / h;
    float uz = dz / h;

    if (!self_i) { H[base_i+0] = -ux; H[base_i+1] = -uy; H[base_i+2] = -uz; }
    if (!self_j) { H[base_j+0] = +ux; H[base_j+1] = +uy; H[base_j+2] = +uz; }

    /* PH = P · H^T   (n×1) — computed from the PRE-update P. */
    float PH[EKF_MAX_STATE];
    for (int i = 0; i < n; i++) {
        float acc = 0.0f;
        for (int j = 0; j < n; j++) acc += ekf.P[i][j] * H[j];
        PH[i] = acc;
    }

    /* S = H · P · H^T + R  (scalar;  R = sigma_d²) */
    float S = sigma_d * sigma_d;
    for (int j = 0; j < n; j++) S += H[j] * PH[j];

    /* Outlier gate — dynamic: uses the full innovation covariance S, not sigma_d. */
    float innov = range_m - h;
    if (fabsf(innov) > EKF_OUTLIER_GATE * sqrtf(S)) return;

    /* Kalman gain K = PH / S  (n×1) */
    float K[EKF_MAX_STATE];
    float S_inv = 1.0f / S;
    for (int i = 0; i < n; i++) K[i] = PH[i] * S_inv;

    /* State update: x += K · innov */
    for (int i = 0; i < n; i++) ekf.x[i] += K[i] * innov;

    /* Covariance update — Joseph form, upper triangle only, then mirrored.
     *
     *   P_new[i][j] = P[i][j] − K[i]·PH[j] − PH[i]·K[j] + S·K[i]·K[j]
     *
     * Safe to write in-place: the formula reads P[i][j] (old value) before
     * writing it, and PH/K are fully precomputed arrays that are not
     * modified here.  The lower-triangle write P[j][i]=v does not affect
     * any upper-triangle read because j > i in the inner loop. */
    for (int i = 0; i < n; i++) {
        for (int j = i; j < n; j++) {
            float v = ekf.P[i][j]
                      - K[i]  * PH[j]
                      - PH[i] * K[j]
                      + S * K[i] * K[j];
            ekf.P[i][j] = ekf.P[j][i] = v;
        }
    }
}

/* ============================================================================
 * ekf_init
 * ============================================================================ */
void ekf_init(void)
{
    memset(&ekf, 0, sizeof(ekf));

    network_t *net    = network_get_network();
    uint16_t   own_id = network_get_ownid();

    for (int i = 0; i < (int)net->count && (int)ekf.n_peers < EKF_MAX_PEERS; i++) {
        uint16_t pid = net->peers[i].id;
        if (pid == 0 || pid == own_id) continue;
        ekf.peer_ids[ekf.n_peers++] = pid;
        /* peer_away / peer_seeded / peer_imu_valid already false from memset */
    }

    int n = (int)ekf.n_peers * 3;
    for (int i = 0; i < n; i++) ekf.P[i][i] = EKF_INIT_P_POS;

    for (int p = 0; p < (int)ekf.n_peers; p++)
        ekf.Q[p*3+2][p*3+2] = EKF_Q_Z_POS;

    ekf.initialised  = false;
    ekf.last_tick_ms = HAL_GetTick();
}

/* ============================================================================
 * ekf_step
 * ============================================================================ */
void ekf_step(float az_self_ms, float ah_self_ms)
{
    static const float origin[3] = {0.0f, 0.0f, 0.0f};

    /* ---- Variable dt -------------------------------------------------------- */
    uint32_t now_ms = HAL_GetTick();
    float dt_s;

    if (!ekf.initialised) {
        /* First step: last_tick_ms was set in ekf_init() just moments ago;
         * the elapsed time is meaningless — use nominal. */
        dt_s = EKF_DT_NOM_S;
    } else {
        /* Unsigned subtraction wraps correctly over the uint32 rollover. */
        dt_s = (float)((uint32_t)(now_ms - ekf.last_tick_ms)) * 0.001f;
        if (dt_s < 0.001f)      dt_s = EKF_DT_NOM_S;   /* guard near-zero jitter */
        if (dt_s > EKF_DT_MAX_S) dt_s = EKF_DT_MAX_S;
    }
    ekf.last_tick_ms = now_ms;

    /* ---- Sync peer list ----------------------------------------------------- */
    sync_peers();

    int n = (int)ekf.n_peers * 3;

    /* Self is always the fixed origin — set unconditionally every step. */
    network_set_self_pos(origin);

    if (n == 0) return;

    /* ---- PREDICT ------------------------------------------------------------ */
    /*
     * F = I (position-only state, no explicit velocity), so P_pred = P + Q.
     * Q is diagonal; only the diagonal of P is updated here.
     * Away peers are skipped: their covariance was already reset when they
     * left, so there is nothing to propagate.
     */
    for (int p = 0; p < (int)ekf.n_peers; p++) {
        if (ekf.peer_away[p]) continue;

        int base = p * 3;

        /* IMU velocity — Bug #2: zero-init of imu_vel_vert decodes to ~−4 m/s
         * in offset-binary encoding.  Use 0 until first confirmed measurement. */
        float peer_vz = 0.0f;
        float peer_vh = 0.0f;
        if (ekf.peer_imu_valid[p]) {
            node_t *peer_node = find_peer(ekf.peer_ids[p]);
            if (peer_node) {
                peer_vz = vel_vert_u8_to_ms(peer_node->imu_vel_vert);
                peer_vh = vel_horiz_u8_to_ms(peer_node->imu_vel_horiz);
            }
        }

        /* Relative Z displacement: peer Z moves at (peer_vz − self_vz) × dt. */
        ekf.x[base + 2] += (peer_vz - az_self_ms) * dt_s;

        /* Horizontal process noise — Bug #10 fix:
         *   Original: h_noise = (peer_vh + ah_self_ms) × dt + floor
         *             This sums speeds linearly and mixes displacement with
         *             variance units.
         *   Fixed:    h_var = (RMS of peer and self speeds)² × dt²  +  floor × (dt/dt_nom)
         *             RMS combines independent random walk contributions.
         *             dt² converts speed² → displacement variance.
         *             floor is a minimum variance floor, scaled by dt ratio. */
        float vh_rms = sqrtf(peer_vh * peer_vh + ah_self_ms * ah_self_ms);
        float h_var  = vh_rms * vh_rms * dt_s * dt_s
                       + EKF_Q_H_FLOOR * (dt_s / EKF_DT_NOM_S);
        float z_var  = EKF_Q_Z_POS * (dt_s / EKF_DT_NOM_S);

        ekf.Q[base+0][base+0] = h_var;
        ekf.Q[base+1][base+1] = h_var;
        ekf.Q[base+2][base+2] = z_var;

        /* P += Q (diagonal only) with per-element cap. */
        ekf.P[base+0][base+0] += h_var;
        if (ekf.P[base+0][base+0] > EKF_P_MAX) ekf.P[base+0][base+0] = EKF_P_MAX;

        ekf.P[base+1][base+1] += h_var;
        if (ekf.P[base+1][base+1] > EKF_P_MAX) ekf.P[base+1][base+1] = EKF_P_MAX;

        ekf.P[base+2][base+2] += z_var;
        if (ekf.P[base+2][base+2] > EKF_P_MAX) ekf.P[base+2][base+2] = EKF_P_MAX;

        /* Bug #14 — EKF_P_MAX diagonal-only cap can break PSD:
         * Clamping only the diagonal while leaving off-diagonal
         * cross-covariances uncapped can violate |P_ij| ≤ sqrt(P_ii·P_jj).
         * A non-PSD P makes S = H·P·H^T + R potentially negative, causing
         * sqrtf(S) to return NaN on Cortex-M4 and corrupting the outlier
         * gate and Kalman gain.  Fix: enforce the Cauchy-Schwarz PSD bound
         * on the 3×3 peer self-block off-diagonals and mirror symmetry.
         * Only 6 element pairs per peer — negligible CPU cost. */
        for (int r = 0; r < 3; r++) {
            for (int c = r + 1; c < 3; c++) {
                float psd_lim = sqrtf(ekf.P[base+r][base+r]
                                    * ekf.P[base+c][base+c]);
                if (ekf.P[base+r][base+c] >  psd_lim)
                    ekf.P[base+r][base+c] =  psd_lim;
                if (ekf.P[base+r][base+c] < -psd_lim)
                    ekf.P[base+r][base+c] = -psd_lim;
                ekf.P[base+c][base+r] = ekf.P[base+r][base+c]; /* symmetry */
            }
        }
    }

    /* ---- UPDATE ------------------------------------------------------------- */
    uint16_t own_id = network_get_ownid();

    /* Category 1: self → each peer -------------------------------------------
     * Self is the fixed origin; only the peer state block is in the Jacobian. */
    for (int p = 0; p < (int)ekf.n_peers; p++) {
        if (ekf.peer_away[p]) continue;

        uint16_t pid = ekf.peer_ids[p];

        node_peer_state_t *ps = network_get_peer_state(own_id, pid);
        if (!ps) continue;

        float range_m = range_scaled_to_m(ps->distance_scaled);
        if (range_m < 0.0f) continue;

        /* Per-peer first-fix seeding — Bug #11 fix:
         *   Original code used a single global 'initialised' flag, so peers
         *   that joined after the first step were never seeded.
         *   Fix: each peer gets its own peer_seeded[] flag. */
        if (!ekf.peer_seeded[p]) {
            /* Bug #13 — Degenerate collinear seeding: all peers placed at
             * (range_m, 0, 0) share y = 0.  That makes the y-component of
             * the Jacobian H[y] = (yj - yi) / h  identically zero for every
             * self→peer measurement, so the Kalman gain has no y-component
             * and y is never updated — it stays 0 regardless of how many
             * nodes are measuring.
             *
             * Fix: place the peer on a circle of radius range_m at a random
             * angle derived from a lightweight xorshift32 hash seeded with
             * run-time entropy (tick, peer_id, slot index).  This produces a
             * non-zero H[y] immediately and lets the filter converge freely
             * in both x and y.  No <stdlib.h> / hardware RNG required.
             */
            {
                uint32_t rng = (uint32_t)(HAL_GetTick()
                                ^ (uint32_t)ekf.peer_ids[p]
                                ^ ((uint32_t)p * 2654435761UL));
                /* One xorshift32 iteration — cheap and sufficient. */
                rng ^= rng << 13;
                rng ^= rng >> 17;
                rng ^= rng << 5;
                /* Map to angle in [0, 2π):  2π / 2^32 ≈ 1.46292e-9 */
                float seed_angle = (float)rng * 1.46291808e-9f;
                ekf.x[p*3+0] = range_m * cosf(seed_angle);
                ekf.x[p*3+1] = range_m * sinf(seed_angle);
                ekf.x[p*3+2] = 0.0f;
            }
            ekf.peer_seeded[p]    = true;
            ekf.peer_imu_valid[p] = true;     /* peer is live; IMU data is valid */
        }

        float sigma_d = ekf_certainty_to_sigma(ps->certainty);
        apply_range_update(0 /* dummy, self_i=true */,
                           p * 3,
                           true  /* self_i */,
                           false /* self_j */,
                           range_m, sigma_d, n);
    }

    /* Category 2: peer → peer ------------------------------------------------
     * Both endpoints are in the joint state; Jacobian spans two blocks.
     * Each pair (pi, pj) processed once (pi < pj); uses peer pi's stored
     * measurement of peer pj as reported by the network layer. */
    for (int pi = 0; pi < (int)ekf.n_peers; pi++) {
        if (ekf.peer_away[pi]) continue;

        for (int pj = pi + 1; pj < (int)ekf.n_peers; pj++) {
            if (ekf.peer_away[pj]) continue;

            node_peer_state_t *ps =
                network_get_peer_state(ekf.peer_ids[pi], ekf.peer_ids[pj]);
            if (!ps) continue;

            float range_m = range_scaled_to_m(ps->distance_scaled);
            if (range_m < 0.0f) continue;

            float sigma_d = ekf_certainty_to_sigma(ps->certainty);
            apply_range_update(pi * 3, pj * 3,
                               false /* self_i */,
                               false /* self_j */,
                               range_m, sigma_d, n);
        }
    }

    ekf.initialised = true;

    /* ---- WRITE-BACK --------------------------------------------------------- */
    network_t *net = network_get_network();
    for (int p = 0; p < (int)ekf.n_peers; p++) {
        if (ekf.peer_away[p]) continue;
        for (int k = 0; k < (int)net->count; k++) {
            if (net->peers[k].id == ekf.peer_ids[p]) {
                net->peers[k].pos[0] = ekf.x[p*3+0];
                net->peers[k].pos[1] = ekf.x[p*3+1];
                net->peers[k].pos[2] = ekf.x[p*3+2];
                break;
            }
        }
    }
}

/* ============================================================================
 * ekf_get_state
 * ============================================================================ */
const coop_ekf_t *ekf_get_state(void)
{
    return &ekf;
}

/* ============================================================================
 * ekf_get_peer_pos
 * ============================================================================ */
bool ekf_get_peer_pos(uint16_t peer_id, float pos_out[3])
{
    int8_t slot = find_ekf_slot(peer_id);
    if (slot < 0) return false;
    pos_out[0] = ekf.x[slot*3 + 0];
    pos_out[1] = ekf.x[slot*3 + 1];
    pos_out[2] = ekf.x[slot*3 + 2];
    return true;
}