#include "ekf.h"
#include "../UWB_app/uwb_network.h"
#include "../Calculations/distance.h"

/* ============================================================================
 * External helpers from your existing codebase
 * ============================================================================ */
extern float  vel_vert_u8_to_ms (uint8_t u);
extern float  vel_horiz_u8_to_ms(uint8_t u);
extern double dist_scale_to_ticks(uint16_t encoded);

/* DW3000: 1 UWB tick ≈ 15.65 ps, speed of light 299 792 458 m/s */
#define UWB_TICKS_PER_METRE  (1.0 / (299792458.0 * 15.65e-12))

static inline float ticks_to_metres(uint16_t distance_scaled)
{
    double ticks = dist_scale_to_ticks(distance_scaled);
    if (ticks < 0.0) return -1.0f;   /* 0xFFFF sentinel = no measurement */
    return (float)(ticks / UWB_TICKS_PER_METRE);
}

/* ============================================================================
 * Single static joint filter instance
 * ============================================================================ */
static coop_ekf_t ekf;

/* ============================================================================
 * Matrix helpers — operate on dynamic sub-blocks of EKF_MAX_STATE arrays.
 * N is the active state dimension (3 * n_peers), determined at runtime.
 * All output arguments must NOT alias any input argument.
 * ============================================================================ */

static void mat_zero_n(float m[EKF_MAX_STATE][EKF_MAX_STATE], int n)
{
    for (int i = 0; i < n; i++)
        memset(m[i], 0, sizeof(float) * n);
}

static void mat_identity_n(float m[EKF_MAX_STATE][EKF_MAX_STATE], int n)
{
    mat_zero_n(m, n);
    for (int i = 0; i < n; i++) m[i][i] = 1.0f;
}

/* ============================================================================
 * ekf_certainty_to_sigma
 * ============================================================================ */
float ekf_certainty_to_sigma(uint8_t certainty)
{
    float q = certainty / 255.0f;
    return EKF_SIGMA_MIN + (EKF_SIGMA_MAX - EKF_SIGMA_MIN) * (1.0f - q);
}

/* ============================================================================
 * Peer index lookup — maps a network peer_id to its slot in ekf.peer_ids[].
 * Returns -1 if not found.
 * ============================================================================ */
static int8_t find_ekf_slot(uint16_t peer_id)
{
    for (int8_t i = 0; i < ekf.n_peers; i++)
        if (ekf.peer_ids[i] == peer_id) return i;
    return -1;
}

/* ============================================================================
 * ekf_init
 * ============================================================================ */
void ekf_init(void)
{
    memset(&ekf, 0, sizeof(ekf));

    network_t *net    = network_get_network();
    uint16_t   own_id = network_get_ownid();

    /* Register all current peers */
    ekf.n_peers = 0;
    for (int i = 0; i < net->count && ekf.n_peers < EKF_MAX_PEERS; i++) {
        if (net->peers[i].id == own_id) continue;
        ekf.peer_ids[ekf.n_peers++] = net->peers[i].id;
    }

    int n = ekf.n_peers * 3;   /* active state dimension */

    /* Initial covariance — large: filter converges from measurements */
    mat_identity_n(ekf.P, n);
    for (int i = 0; i < n; i++) ekf.P[i][i] = EKF_INIT_P_POS;

    /* Process noise — Z blocks small (vertical known), X/Y set per-step */
    mat_zero_n(ekf.Q, n);
    for (int p = 0; p < ekf.n_peers; p++)
        ekf.Q[p*3 + 2][p*3 + 2] = EKF_Q_Z_POS;

    ekf.initialised = false;
}

/* ============================================================================
 * sync_peers
 *   Checks whether any new peers have appeared in the network since init.
 *   New peers are appended to the joint state with large uncertainty.
 *   (Peer removal is not handled here — call ekf_init() for that.)
 * ============================================================================ */
static void sync_peers(void)
{
    network_t *net    = network_get_network();
    uint16_t   own_id = network_get_ownid();

    for (int i = 0; i < net->count; i++) {
        uint16_t pid = net->peers[i].id;
        if (pid == own_id) continue;
        if (find_ekf_slot(pid) >= 0) continue;       /* already tracked */
        if (ekf.n_peers >= EKF_MAX_PEERS) continue;  /* no room         */

        int slot = ekf.n_peers++;
        ekf.peer_ids[slot] = pid;

        int base = slot * 3;
        ekf.P[base+0][base+0] = EKF_INIT_P_POS;
        ekf.P[base+1][base+1] = EKF_INIT_P_POS;
        ekf.P[base+2][base+2] = EKF_INIT_P_POS;
        ekf.Q[base+2][base+2] = EKF_Q_Z_POS;
    }
}

/* ============================================================================
 * apply_range_update
 *   Sequential EKF update for one range measurement between two endpoints.
 *
 *   base_i : state base index of node i  (0 for self, peer_slot*3 for peers)
 *   base_j : state base index of node j
 *   self_i : true if node i is self (pos fixed at [0,0,0], not in state)
 *   self_j : true if node j is self
 *   range_m: measured distance (metres)
 *   sigma_d: measurement noise std-dev (metres)
 *
 *   The Jacobian H is a 1×n sparse vector:
 *     For node i (if not self): H[base_i .. base_i+2] = +unit vector i→j
 *     For node j (if not self): H[base_j .. base_j+2] = -unit vector i→j
 *   (derivative of ||xi - xj|| w.r.t. each position component)
 * ============================================================================ */
static void apply_range_update(int base_i, int base_j,
                               bool self_i, bool self_j,
                               float range_m, float sigma_d,
                               int n)
{
    /* Position of node i */
    float xi = self_i ? 0.0f : ekf.x[base_i + 0];
    float yi = self_i ? 0.0f : ekf.x[base_i + 1];
    float zi = self_i ? 0.0f : ekf.x[base_i + 2];

    /* Position of node j */
    float xj = self_j ? 0.0f : ekf.x[base_j + 0];
    float yj = self_j ? 0.0f : ekf.x[base_j + 1];
    float zj = self_j ? 0.0f : ekf.x[base_j + 2];

    /* Predicted range */
    float dx = xj - xi;
    float dy = yj - yi;
    float dz = zj - zi;
    float h  = sqrtf(dx*dx + dy*dy + dz*dz);
    if (h < 1e-4f) h = 1e-4f;

    /* Outlier gate */
    float innov = range_m - h;
    if (fabsf(innov) > EKF_OUTLIER_GATE * sigma_d) return;

    /* Build sparse H vector (1×n) */
    float H[EKF_MAX_STATE];
    memset(H, 0, sizeof(float) * n);

    /* Unit vector from i to j */
    float ux = dx / h, uy = dy / h, uz = dz / h;

    /* d(||xi-xj||)/d(xi) = -(xj-xi)/h = -ux  — node i block */
    if (!self_i) {
        H[base_i + 0] = -ux;
        H[base_i + 1] = -uy;
        H[base_i + 2] = -uz;
    }
    /* d(||xi-xj||)/d(xj) = +(xj-xi)/h = +ux  — node j block */
    if (!self_j) {
        H[base_j + 0] = +ux;
        H[base_j + 1] = +uy;
        H[base_j + 2] = +uz;
    }

    /* PH = P * H^T  (Nx1) */
    float PH[EKF_MAX_STATE];
    for (int i = 0; i < n; i++) {
        PH[i] = 0.0f;
        for (int j = 0; j < n; j++)
            PH[i] += ekf.P[i][j] * H[j];
    }

    /* S = H*P*H^T + R  (scalar) */
    float S = sigma_d * sigma_d;
    for (int j = 0; j < n; j++) S += H[j] * PH[j];

    /* K = PH / S  (Nx1) */
    float K[EKF_MAX_STATE];
    for (int i = 0; i < n; i++) K[i] = PH[i] / S;

    /* State update: x += K * innov */
    for (int i = 0; i < n; i++) ekf.x[i] += K[i] * innov;

    /* Covariance update: P = P - K * (PH)^T
     * Since P is symmetric, HP = (PH^T)^T, so K*H*P = outer(K, PH).
     * Update is done in-place — no temp matrices needed. */
    for (int i = 0; i < n; i++)
        for (int j = 0; j < n; j++)
            ekf.P[i][j] -= K[i] * PH[j];
}

/* ============================================================================
 * ekf_step
 * ============================================================================ */
void ekf_step(float az_self_ms, float ah_self_ms)
{
    network_t *net    = network_get_network();

    /* Always sync in case new peers joined since last init */
    sync_peers();

    int n = ekf.n_peers * 3;
    if (n == 0) return;

    /* ------------------------------------------------------------------
     * PREDICT
     * Each peer's position shifts by its relative velocity w.r.t. self.
     * Horizontal direction unknown for both: add magnitudes for worst-case.
     * Vertical direction known: subtract signed values.
     * ------------------------------------------------------------------ */
    /* F = I (position-only state), so P = F*P*F^T + Q simplifies to P += Q.
     * No F matrix needed. Update state and Q per peer: */
    for (int p = 0; p < ekf.n_peers; p++) {
        int base = p * 3;

        node_t *peer_node = NULL;
        for (int i = 0; i < net->count; i++) {
            if (net->peers[i].id == ekf.peer_ids[p]) {
                peer_node = &net->peers[i];
                break;
            }
        }
        if (!peer_node) continue;

        float peer_vz = vel_vert_u8_to_ms (peer_node->imu_vel_vert);
        float peer_vh = vel_horiz_u8_to_ms(peer_node->imu_vel_horiz);

        float dz = (peer_vz - az_self_ms) * 0.2f;   /* relative Z displacement: dt=0.2s */
        float dh = (peer_vh + ah_self_ms) * 0.2f;   /* relative horizontal magnitude     */

        /* Displace Z state */
        ekf.x[base + 2] += dz;

        /* Dynamic process noise */
        float h_noise = dh + EKF_Q_H_FLOOR;
        ekf.Q[base + 0][base + 0] = h_noise;
        ekf.Q[base + 1][base + 1] = h_noise;
        ekf.Q[base + 2][base + 2] = EKF_Q_Z_POS;
    }

    /* P = F*P*F^T + Q  (F=I so P = P + Q) */
    for (int i = 0; i < n; i++)
        for (int j = 0; j < n; j++)
            ekf.P[i][j] += ekf.Q[i][j];

    /* ------------------------------------------------------------------
     * UPDATE — iterate all inter-node distance pairs
     *
     * Two categories:
     *   1. Self → peer   : anchor=[0,0,0] (self), tag=peer state block
     *   2. Peer → peer   : both endpoints in joint state
     *
     * Distance source: each node's net.self.peers[] slot holds distances
     * that node measured. On node A:
     *   net.self.peers[j] holds A→peerJ distances.
     * For peer-peer distances (B→C) we look in net.peers[B].peers[slot_C].
     * ------------------------------------------------------------------ */

    /* Category 1: self → each peer */
    for (int p = 0; p < ekf.n_peers; p++) {
        uint16_t pid = ekf.peer_ids[p];

        /* Find self's ranging slot for this peer */
        uint16_t dist_scaled = 0xFFFF;
        uint8_t  certainty   = 0;
        for (uint8_t j = 0; j < NETWORK_MAX_PEERS; j++) {
            if (net->self.peers[j].peer_id == pid) {
                dist_scaled = net->self.peers[j].distance_scaled;
                certainty   = net->self.peers[j].certainty;
                break;
            }
        }

        float range_m = ticks_to_metres(dist_scaled);
        if (range_m < 0.0f) continue;

        /* First valid fix: seed position at (range, 0, 0) */
        if (!ekf.initialised) {
            ekf.x[p*3 + 0] = range_m;
            ekf.x[p*3 + 1] = 0.0f;
            ekf.x[p*3 + 2] = 0.0f;
        }

        float sigma_d = ekf_certainty_to_sigma(certainty);
        apply_range_update(
            0,      /* base_i: self has no state block — dummy, self_i=true */
            p * 3,  /* base_j: this peer's state block                      */
            true,   /* self_i: self is fixed at origin                      */
            false,  /* self_j                                                */
            range_m, sigma_d, n);
    }
    ekf.initialised = true;

    /* Category 2: peer → peer (use distances stored in each peer's node_t) */
    for (int pi = 0; pi < ekf.n_peers; pi++) {
        uint16_t id_i = ekf.peer_ids[pi];

        /* Find this peer's node in the network */
        node_t *node_i = NULL;
        for (int k = 0; k < net->count; k++) {
            if (net->peers[k].id == id_i) { node_i = &net->peers[k]; break; }
        }
        if (!node_i) continue;

        for (int pj = pi + 1; pj < ekf.n_peers; pj++) {
            uint16_t id_j = ekf.peer_ids[pj];

            /* Find node_i's stored distance to peer j */
            uint16_t dist_scaled = 0xFFFF;
            uint8_t  certainty   = 0;
            for (uint8_t s = 0; s < NETWORK_MAX_PEERS; s++) {
                if (node_i->peers[s].peer_id == id_j) {
                    dist_scaled = node_i->peers[s].distance_scaled;
                    certainty   = node_i->peers[s].certainty;
                    break;
                }
            }

            float range_m = ticks_to_metres(dist_scaled);
            if (range_m < 0.0f) continue;

            float sigma_d = ekf_certainty_to_sigma(certainty);
            apply_range_update(
                pi * 3,   /* base_i: peer i's state block */
                pj * 3,   /* base_j: peer j's state block */
                false,    /* self_i                       */
                false,    /* self_j                       */
                range_m, sigma_d, n);
        }
    }

    /* ------------------------------------------------------------------
     * Write positions back to network layer
     * ------------------------------------------------------------------ */
    for (int p = 0; p < ekf.n_peers; p++) {
        uint16_t pid = ekf.peer_ids[p];
        for (int k = 0; k < net->count; k++) {
            if (net->peers[k].id == pid) {
                net->peers[k].pos[0] = ekf.x[p*3 + 0];
                net->peers[k].pos[1] = ekf.x[p*3 + 1];
                net->peers[k].pos[2] = ekf.x[p*3 + 2];
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