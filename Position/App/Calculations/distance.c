#include "cmsis_os.h"
#include "cmsis_os2.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

#include "../Generic/my_print.h"
#include "DWM3000_setup.h"
#include "DWM3000_driver.h"
#include "../UWB_app/messages.h"
#include "../UWB_app/uwb_network.h"
#include "../UWB_app/uwb_exchange.h"
#include "distance.h"
#include "lsm6dsv.h"

timestamps_t timestamps;

/*
 * k_ema_update() — EMA smooth a clock-ratio estimate into *k.
 *
 * alpha: how much weight to give the NEW sample.
 *   Higher alpha → faster adaptation, less smoothing.
 *   Recommended values:
 *     Second-order (direct SS-TWR):  0.20  — trusted, adapt quickly
 *     Third-order  (TDOA derived):   0.05  — less trusted, smooth heavily
 *
 * Seeding rule: if *k is exactly 1.0 (uninitialised default),
 * the first sample is taken directly without blending.
 */
static void k_ema_update(double *k, double k_new, double alpha)
{
    if (*k == 1.0)
        *k = k_new;                              /* first-call seed */
    else
        *k += alpha * (k_new - *k);              /* EMA: y += α*(x-y) */
}

static inline uint8_t vel_vert_to_u8(float v)
{
    float c = v < -VEL_VERT_RANGE_MS ? -VEL_VERT_RANGE_MS
            : v >  VEL_VERT_RANGE_MS ?  VEL_VERT_RANGE_MS : v;
    return (uint8_t)((c + VEL_VERT_RANGE_MS) / VEL_VERT_MS_PER_LSB + 0.5f);
}

static inline float vel_vert_u8_to_ms(uint8_t u)
{
    return u * VEL_VERT_MS_PER_LSB - VEL_VERT_RANGE_MS;
}

static inline uint8_t vel_horiz_to_u8(float v)
{
    /* take absolute value — direction not used */
    float c = v < 0.0f ? -v : v;
    if (c > VEL_HORIZ_MAX_MS) c = VEL_HORIZ_MAX_MS;
    return (uint8_t)(c / VEL_HORIZ_MS_PER_LSB + 0.5f);
}

static inline float vel_horiz_u8_to_ms(uint8_t u)
{
    return u * VEL_HORIZ_MS_PER_LSB;   /* always positive */
}

void populate_computation_structs(timestamps_t *ts) {
    if (!ts) return;

    network_t *net = network_get_network();
    if (!net) return;

    uint8_t peer_count = 0;
    const node_t *peers = network_get_peers(&peer_count);

    memset(ts, 0, sizeof(timestamps_t));
    const measurements_t *meas = &net->measurements;

    if (!peers || peer_count == 0) return;

    uint16_t initiator_id = network_get_master();
    uint16_t responder_id = network_get_ownid();

    /* ── Read own IMU → encode to uint8_t → store into network state ── */
    uint32_t flags = osThreadFlagsWait(0x02, osFlagsWaitAll, 2);
    float pitch_rad = 0, speed_horiz = 0, vel_z = 0;
    if (!(flags & 0x80000000U) && (flags & 0x02U))
        imu_get_results(&pitch_rad, &speed_horiz, &vel_z);

    net->self.imu_vel_vert  = vel_vert_to_u8(vel_z);        /* float m/s → uint8_t */
    net->self.imu_vel_horiz = vel_horiz_to_u8(speed_horiz);  /* float m/s → uint8_t */

    node_t *init_node = find_peer(initiator_id);
    if (init_node) {
        init_node->imu_vel_vert  = vel_vert_to_u8(meas->final.IMU_vel_vert);
        init_node->imu_vel_horiz = vel_horiz_to_u8(meas->final.IMU_vel_horiz);
    }

    /* -----------------------------------------------------------------------
     * 1. FIRST ORDER: Initiator (A) and Responder (Self/B)
     *    vel fields in timestamps_t are float — decode uint8_t from wire back
     *    to float for computation, keep own IMU as raw float.
     * ----------------------------------------------------------------------- */
    ts->first.initiator_id        = initiator_id;
    ts->first.responder_id        = responder_id;

    ts->first.twr.poll_rx         = meas->poll_rx;
    ts->first.twr.resp_tx         = meas->resp_tx;
    ts->first.twr.final_rx        = meas->final_rx;
    ts->first.twr.poll_tx         = meas->final.poll_tx_ts;
    ts->first.twr.resp_rx.ts      = meas->final.resp_rx_ts;
    ts->first.twr.resp_rx.pwr_diff_q8 = meas->final.resp_pwr_diff_q8;
    ts->first.twr.final_tx        = meas->final.final_tx_ts;


    /* -----------------------------------------------------------------------
     * 2. SECOND ORDER
     * ----------------------------------------------------------------------- */
    uint8_t p_count = meas->passive_count;
    for (uint8_t i = 0; i < p_count; i++) {
        uint16_t passive_id = meas->passive_device_id[i];

        twr_observation_t shared_obs = {
            .poll_rx = {
                .ts      = meas->passive[i].poll_rx_ts,
                .pwr_diff_q8 = meas->passive[i].poll_pwr_diff_q8,
            },
            .resp_rx = {
                .ts      = meas->passive[i].resp_rx_ts,
                .pwr_diff_q8 = meas->passive[i].resp_pwr_diff_q8,
            },
        };

        uwb_rx_meas_t a_rx_of_passive = {0};
        for (uint8_t k = 0; k < meas->final.entry_count; k++) {
            if (meas->final.entry_id[k] == passive_id) {
                a_rx_of_passive.ts      = meas->final.entries[k];
                a_rx_of_passive.pwr_diff_q8 = meas->final.entry_pwr_diff_q8[k];
                break;
            }
        }

        /* Passive peer IMU: msg_passive_t carries float → encode to uint8_t for node_t */
        node_t *passive_node = find_peer(passive_id);
        if (passive_node) {
            passive_node->imu_vel_vert  = vel_vert_to_u8(meas->passive[i].IMU_vel_vert);
            passive_node->imu_vel_horiz = vel_horiz_to_u8(meas->passive[i].IMU_vel_horiz);
        }

        if ((int)ts->second_count + 2 > MAX_SECOND_ORDER) { 
            mprintf("[ERR] populate: second_order overflow at passive %u\n", i);
            break;
        }

        /* A <-> Passive */
        second_order_t *so_a      = &ts->second[ts->second_count++];
        so_a->initiator_id        = initiator_id;
        so_a->responder_id        = passive_id;
        so_a->twr_observation     = shared_obs;
        so_a->twr.init_tx         = meas->final.poll_tx_ts;
        so_a->twr.init_rx         = shared_obs.poll_rx;
        so_a->twr.answer_tx       = meas->passive[i].passive_tx_ts;
        so_a->twr.answer_rx       = a_rx_of_passive;

        /* B <-> Passive */
        second_order_t *so_b      = &ts->second[ts->second_count++];
        so_b->initiator_id        = responder_id;
        so_b->responder_id        = passive_id;
        so_b->twr_observation     = shared_obs;
        so_b->twr.init_tx         = meas->resp_tx;
        so_b->twr.init_rx         = shared_obs.resp_rx;
        so_b->twr.answer_tx       = meas->passive[i].passive_tx_ts;
        so_b->twr.answer_rx       = meas->passive_rx[i];
    }

    /* -----------------------------------------------------------------------
     * 3. THIRD ORDER: Passive-to-Passive
     * ----------------------------------------------------------------------- */
    for (uint8_t i = 0; i < p_count; i++) {
    for (uint8_t j = i + 1; j < p_count; j++) {

        if (ts->third_count >= MAX_THIRD_ORDER) {
            mprintf("[ERR] populate: third_order overflow at pair (%u,%u)\n", i, j);
            break;
        }

        third_order_t *to       = &ts->third[ts->third_count++];
        to->initiator_id        = meas->passive_device_id[i]; /* C */
        to->responder_id        = meas->passive_device_id[j]; /* D */


        /* C's perspective of the A-B exchange */
        to->twr_observation_c.poll_rx.ts          = meas->passive[i].poll_rx_ts;
        to->twr_observation_c.poll_rx.pwr_diff_q8 = meas->passive[i].poll_pwr_diff_q8;
        to->twr_observation_c.resp_rx.ts          = meas->passive[i].resp_rx_ts;
        to->twr_observation_c.resp_rx.pwr_diff_q8 = meas->passive[i].resp_pwr_diff_q8;

        /* D's perspective of the A-B exchange */
        to->twr_observation_d.poll_rx.ts          = meas->passive[j].poll_rx_ts;
        to->twr_observation_d.poll_rx.pwr_diff_q8 = meas->passive[j].poll_pwr_diff_q8;
        to->twr_observation_d.resp_rx.ts          = meas->passive[j].resp_rx_ts;
        to->twr_observation_d.resp_rx.pwr_diff_q8 = meas->passive[j].resp_pwr_diff_q8;

        /* ss_twr_t: C transmits (init_tx), D receives (answer_rx).
        * init_rx = D's reception of C's passive TX (from D's entry list).
        * answer_tx = D's passive TX timestamp. */
        to->twr.init_tx   = meas->passive[i].passive_tx_ts;  /* C_tx */
        to->twr.init_rx   = (uwb_rx_meas_t){0};              /* C_rx not available */

        /* Find D's entry for C */
        to->twr.answer_rx = (uwb_rx_meas_t){0};
        for (uint8_t k = 0; k < meas->passive[j].entry_count; k++) {
            if (meas->passive[j].entry_ids[k] == meas->passive_device_id[i]) {
                to->twr.answer_rx.ts           = meas->passive[j].entries[k];
                to->twr.answer_rx.pwr_diff_q8  = meas->passive[j].entry_pwr_diff_q8[k];
                break;
            }
        }

        to->twr.answer_tx = meas->passive[j].passive_tx_ts;  /* D_tx */
    }
    }
}

#ifdef DEBUG_distance_populate
/* Since populate() always calls memset(ts, 0, ...) first,
 * any field still zero after populate() was never written. */
static bool check_uwb_rx_meas(const uwb_rx_meas_t *m, const char *name)
{
    if (m->ts == 0) {
        /* memset guarantees: ts==0 means populate() never touched this field */
        mprintf("[FAIL] %s.ts == 0 (unpopulated)\n", name);
        return false;
    }
    if (m->pwr_diff_q8 == 0) {
        /* 0 dB is physically possible, but suspicious — warn only */
        mprintf("[WARN] %s.pwr_diff_q8 == 0 (0 dB — check if intentional)\n", name);
    }
    return true;
}

static bool check_twr_timestamps(const twr_timestamps_t *t, const char *pfx)
{
    bool ok = true;
    char buf[48];

    /* All tx times must be non-zero: hardware never schedules at tick 0 */
#define CHK_TX(field) \
    if (t->field == 0) { mprintf("[FAIL] %s." #field " == 0 (unpopulated)\n", pfx); ok = false; }

    CHK_TX(poll_tx);
    snprintf(buf, sizeof(buf), "%s.poll_rx",  pfx); ok &= check_uwb_rx_meas(&t->poll_rx,  buf);
    CHK_TX(resp_tx);
    snprintf(buf, sizeof(buf), "%s.resp_rx",  pfx); ok &= check_uwb_rx_meas(&t->resp_rx,  buf);
    CHK_TX(final_tx);
    snprintf(buf, sizeof(buf), "%s.final_rx", pfx); ok &= check_uwb_rx_meas(&t->final_rx, buf);
#undef CHK_TX
    return ok;
}

static bool check_twr_observation(const twr_observation_t *o, const char *pfx)
{
    bool ok = true;
    char buf[48];
    snprintf(buf, sizeof(buf), "%s.poll_rx", pfx); ok &= check_uwb_rx_meas(&o->poll_rx, buf);
    snprintf(buf, sizeof(buf), "%s.resp_rx", pfx); ok &= check_uwb_rx_meas(&o->resp_rx, buf);
    return ok;
}

static bool check_ss_twr(const ss_twr_t *s, const char *pfx)
{
    bool ok = true;
    char buf[48];
    if (s->init_tx == 0)   { mprintf("[FAIL] %s.init_tx == 0 (unpopulated)\n",   pfx); ok = false; }
    snprintf(buf, sizeof(buf), "%s.init_rx",   pfx); ok &= check_uwb_rx_meas(&s->init_rx,   buf);
    if (s->answer_tx == 0) { mprintf("[FAIL] %s.answer_tx == 0 (unpopulated)\n", pfx); ok = false; }
    snprintf(buf, sizeof(buf), "%s.answer_rx", pfx); ok &= check_uwb_rx_meas(&s->answer_rx, buf);
    return ok;
}

/* ── per-order checkers ─────────────────────────────────────────── */

static bool check_first_order(const first_order_t *f)
{
    bool ok = true;
    /* IDs start at 0 after memset — if still 0, they were never assigned */
    if (f->initiator_id == 0) { mprintf("[FAIL] first.initiator_id == 0\n"); ok = false; }
    if (f->responder_id == 0) { mprintf("[FAIL] first.responder_id == 0\n"); ok = false; }

    mprintf("[TEST] first: init_id=%u resp_id=%u\n",
            f->initiator_id, f->responder_id);

    ok &= check_twr_timestamps(&f->twr, "first.twr");
    mprintf("[TEST] first_order: %s\n", ok ? "OK" : "FAIL");
    return ok;
}

static bool check_second_order(const second_order_t *s, uint8_t idx)
{
    bool ok = true;
    char pfx[24], buf[32];
    snprintf(pfx, sizeof(pfx), "second[%u]", idx);

    if (s->initiator_id == 0) { mprintf("[FAIL] %s.initiator_id == 0\n", pfx); ok = false; }
    if (s->responder_id == 0) { mprintf("[FAIL] %s.responder_id == 0\n", pfx); ok = false; }

    snprintf(buf, sizeof(buf), "%s.obs", pfx); ok &= check_twr_observation(&s->twr_observation, buf);
    snprintf(buf, sizeof(buf), "%s.twr", pfx); ok &= check_ss_twr(&s->twr, buf);
    return ok;
}

static bool check_third_order(const third_order_t *t, uint8_t idx)
{
    bool ok = true;
    char pfx[24], buf[32];
    snprintf(pfx, sizeof(pfx), "third[%u]", idx);

    if (t->initiator_id == 0) { mprintf("[FAIL] %s.initiator_id == 0\n", pfx); ok = false; }
    if (t->responder_id == 0) { mprintf("[FAIL] %s.responder_id == 0\n", pfx); ok = false; }

    snprintf(buf, sizeof(buf), "%s.obs", pfx); ok &= check_twr_observation(&t->twr_observation, buf);
    snprintf(buf, sizeof(buf), "%s.twr", pfx); ok &= check_ss_twr(&t->twr, buf);
    return ok;
}

/* ── top-level test ─────────────────────────────────────────────── */

bool test_timestamps_populated(const timestamps_t *ts)
{
    bool ok = true;
    mprintf("[TEST] ===== timestamps_t population check =====\n");

    ok &= check_first_order(&ts->first);

    /* second_count: memset zeroed it, so 0 now means populate() never set it */
    const uint8_t max_second = 2u * (NETWORK_MAX_PEERS - 1u);
    if (ts->second_count == 0) {
        mprintf("[FAIL] second_count == 0 — populate() never wrote it\n");
        ok = false;
    } else if (ts->second_count > max_second) {
        mprintf("[FAIL] second_count=%u > max %u (corruption?)\n",
                ts->second_count, max_second);
        ok = false;
    } else {
        mprintf("[TEST] second_count=%u / %u\n", ts->second_count, max_second);
        bool sec_ok = true;
        for (uint8_t i = 0; i < ts->second_count; i++) {
            sec_ok &= check_second_order(&ts->second[i], i);
        }
        mprintf("[TEST] second entries: %s\n", sec_ok ? "OK" : "FAIL");
        ok &= sec_ok;
    }

    /* third_count == 0 is valid when NETWORK_MAX_PEERS <= 2, so warn only */
    const uint8_t max_third =
        (uint8_t)((NETWORK_MAX_PEERS - 1u) * (NETWORK_MAX_PEERS - 2u) / 2u);
    if (ts->third_count > max_third) {
        mprintf("[FAIL] third_count=%u > max %u (corruption?)\n",
                ts->third_count, max_third);
        ok = false;
    } else if (ts->third_count == 0 && max_third > 0) {
        mprintf("[WARN] third_count == 0 but max is %u — expected?\n", max_third);
        /* Not a hard fail — topology may have no third-order pairs */
    } else {
        mprintf("[TEST] third_count=%u / %u\n", ts->third_count, max_third);
        bool trd_ok = true;
        for (uint8_t i = 0; i < ts->third_count; i++) {
            trd_ok &= check_third_order(&ts->third[i], i);
        }
        mprintf("[TEST] third entries: %s\n", trd_ok ? "OK" : "FAIL");
        ok &= trd_ok;
    }

    mprintf("[TEST] ===== RESULT: %s =====\n", ok ? "PASS" : "FAIL");
    return ok;
}
#endif /* DEBUG_distance_populate */

static void calculate_first_order_distance_ticks(first_order_t *f)
{
    if (!f) {
        return;
    }

    const twr_timestamps_t *t = &f->twr;

    uint64_t T_round1 = (t->resp_rx.ts  - t->poll_tx)    & UWB_40BIT_MASK;
    uint64_t T_reply1 = (t->resp_tx     - t->poll_rx.ts) & UWB_40BIT_MASK;
    uint64_t T_round2 = (t->final_rx.ts - t->resp_tx)    & UWB_40BIT_MASK;
    uint64_t T_reply2 = (t->final_tx    - t->resp_rx.ts) & UWB_40BIT_MASK;

    mprintf("[TWR] T_round1=%u T_reply1=%u T_round2=%u T_reply2=%u\n",
            (uint32_t)T_round1, (uint32_t)T_reply1,
            (uint32_t)T_round2, (uint32_t)T_reply2);

    /* 1. Zero-interval guard — unpopulated timestamps */
    if (T_round1 == 0 || T_reply1 == 0 || T_round2 == 0 || T_reply2 == 0) {
        mprintf("[ERR] first_order: zero interval — unpopulated timestamp\n");
        f->result_distance_tick = -1.0;
        return;
    }

    /* 2. Same-domain constraint: both on the responder's clock */
    int64_t diff1 = (int64_t)T_round1 - (int64_t)T_reply1;
    if (diff1 < -500) {
        mprintf("[ERR] first_order: T_reply1(%u) massively exceeds T_round1(%u)\n",
                (uint32_t)T_reply1, (uint32_t)T_round1);
        f->result_distance_tick = -1.0;
        return;
    } else if (diff1 < 0) {
        T_round1 = T_reply1; /* Clamp to 0 distance for this segment */
    }

    int64_t diff2 = (int64_t)T_round2 - (int64_t)T_reply2;
    if (diff2 < -500) {
        mprintf("[ERR] first_order: T_reply2 massively exceeds T_round2\n");
        f->result_distance_tick = -1.0;
        return;
    } else if (diff2 < 0) {
        T_round2 = T_reply2; /* Clamp to 0 distance for this segment */
    }

    /* 3. DS-TWR formula — each cast individually, sum stays in double */
    double dR1 = (double)T_round1;
    double dR2 = (double)T_round2;
    double dP1 = (double)T_reply1;
    double dP2 = (double)T_reply2;

    double den = dR1 + dR2 + dP1 + dP2;
    if (den == 0.0) {
        mprintf("[ERR] first_order: zero denominator\n");
        f->result_distance_tick = -1.0;
        return;
    }

    double tof_ticks = (dR1 * dR2 - dP1 * dP2) / den;

    /* 4. Sanity — reject catastrophic cancellation and physically impossible values */
    if (tof_ticks < -200.0) {
        mprintf("[ERR] first_order: tof=%.1f ticks — catastrophic cancellation, check final_tx latch\n",
                tof_ticks);
        f->result_distance_tick = -1.0;
        return;
    }

    f->result_distance_tick = tof_ticks;
    return;
}

static void calculate_second_order_distances(
    second_order_t      *so_a,
    second_order_t      *so_b,
    const first_order_t *first,
    double tof_ac_init, double tof_bc_init,
    double *k_A, double *k_B)
{
    so_a->result_distance_tick = -1.0;
    so_b->result_distance_tick = -1.0;

    if (!so_a || !so_b || !first || !k_A || !k_B) return;

    if (so_a->responder_id != so_b->responder_id) {
        mprintf("[ERR] second_order: passive ID mismatch (%u vs %u)\n",
                so_a->responder_id, so_b->responder_id);
        return;
    }
    if (so_a->initiator_id != first->initiator_id ||
        so_b->initiator_id != first->responder_id) {
        mprintf("[ERR] second_order: initiator IDs don't match first-order — entries swapped?\n");
        return;
    }

    /* Read A-B ToF from the already-computed first-order result */
    double tof_ab_ticks = first->result_distance_tick;
    if (tof_ab_ticks < 0.0) {
        mprintf("[ERR] second_order: first-order result not available\n");
        return;
    }

    /* ── shared references ─────────────────────────────── */
    const twr_timestamps_t  *ft  = &first->twr;
    const twr_observation_t *obs = &so_a->twr_observation;   /* shared C observation */

    /* ── passive C observation window ──────────────────── */
    uint64_t dt_C = (obs->resp_rx.ts - obs->poll_rx.ts) & UWB_40BIT_MASK;
    if (dt_C == 0) { mprintf("[ERR] second_order: dt_C == 0\n"); return; }

    /* ── SS-TWR intervals for A-C and B-C ──────────────── */
    uint64_t T_round_AC = (so_a->twr.answer_rx.ts - so_a->twr.init_tx)    & UWB_40BIT_MASK;
    uint64_t T_reply_AC = (so_a->twr.answer_tx    - so_a->twr.init_rx.ts) & UWB_40BIT_MASK;
    uint64_t T_round_BC = (so_b->twr.answer_rx.ts - so_b->twr.init_tx)    & UWB_40BIT_MASK;
    uint64_t T_reply_BC = (so_b->twr.answer_tx    - so_b->twr.init_rx.ts) & UWB_40BIT_MASK;

    /* ── step-1 reference intervals ────────────────────── */
    double T_round1_A = (double)((ft->resp_rx.ts - ft->poll_tx) & UWB_40BIT_MASK);
    double dt_A_ref   = T_round1_A - tof_ab_ticks;
    double dt_B_ref   = (double)((ft->resp_tx - ft->poll_rx.ts) & UWB_40BIT_MASK);

    if (tof_ab_ticks > T_round1_A + 500.0) {
        mprintf("[ERR] second_order: tof_ab_ticks massively exceeds T_round1_A\n");
        return;
    } else if (tof_ab_ticks >= T_round1_A) {
        tof_ab_ticks = T_round1_A - 0.1;
        dt_A_ref = 0.1;
    }

    if (dt_B_ref < -500.0) {
        mprintf("[ERR] second_order: massively negative dt_B_ref\n");
        return;
    } else if (dt_B_ref <= 0.0) {
        dt_B_ref = 0.1;
    }

    /* ═══════════════════════════════════════════════════
     * STEP 1 — geometric estimate (always runs)
     * If a prior is available, use it to seed dt_A_ref /
     * dt_B_ref with a correction before the geometry pass,
     * giving a better baseline on the first iteration.
     * ═══════════════════════════════════════════════════ */
    const bool have_prior = (tof_ac_init > 0.0 && tof_bc_init > 0.0);

    /* Seed correction from prior if available, else zero */
    double correction_s1 = have_prior ? (tof_bc_init - tof_ac_init) : 0.0;

    /* Geometric pass using seeded correction */
    double dt_A_ref_s1 = dt_A_ref + correction_s1;
    double dt_B_ref_s1 = dt_B_ref + tof_ab_ticks + correction_s1;

    /* Clamp to avoid division by zero / negative k */
    if (dt_A_ref_s1 <= 0.0) dt_A_ref_s1 = 0.1;
    if (dt_B_ref_s1 <= 0.0) dt_B_ref_s1 = 0.1;

    double k_A_s1    = dt_A_ref_s1 / (double)dt_C;
    double k_B_s1    = dt_B_ref_s1 / (double)dt_C;
    double tof_ac_s1 = ((double)T_round_AC - k_A_s1 * (double)T_reply_AC) / 2.0;
    double tof_bc_s1 = ((double)T_round_BC - k_B_s1 * (double)T_reply_BC) / 2.0;

    mprintf("[STEP1] A(%u)<->C(%u): %.1f ticks  B(%u)<->C(%u): %.1f ticks  (%s)\n",
            so_a->initiator_id, so_a->responder_id, tof_ac_s1,
            so_b->initiator_id, so_b->responder_id, tof_bc_s1,
            have_prior ? "prior-seeded" : "geometric");

    if (tof_ac_s1 < -200.0 || tof_bc_s1 < -200.0) {
        mprintf("[ERR] second_order: step1 ToF implausibly negative (ac=%.1f bc=%.1f)\n",
                tof_ac_s1, tof_bc_s1);
        return;
    }

    /* Step-1 result becomes the correction for step-2 */
    correction_s1 = tof_bc_s1 - tof_ac_s1;
    

    /* ═══════════════════════════════════════════════════
     * STEP 2 — corrected k using best available correction
     * ═══════════════════════════════════════════════════ */
    double dt_A_ref_corr = dt_A_ref + correction_s1;
    double dt_B_ref_corr = dt_B_ref + tof_ab_ticks + correction_s1;

    if (dt_A_ref_corr <= 0.0 || dt_B_ref_corr <= 0.0) {
        mprintf("[WARN] second_order: corrected reference <= 0 — falling back to step-1 geometry\n");
        double k_A_s1 = dt_A_ref / (double)dt_C;
        double k_B_s1 = (dt_B_ref + tof_ab_ticks) / (double)dt_C;
        so_a->result_distance_tick = ((double)T_round_AC - k_A_s1 * (double)T_reply_AC) / 2.0;
        so_b->result_distance_tick = ((double)T_round_BC - k_B_s1 * (double)T_reply_BC) / 2.0;
        return;
    }

    double k_A_new = dt_A_ref_corr / (double)dt_C;
    double k_B_new = dt_B_ref_corr / (double)dt_C;

    double drift_A = fabs(k_A_new - 1.0) * 1e6;
    double drift_B = fabs(k_B_new - 1.0) * 1e6;
    if (drift_A > 500.0 || drift_B > 500.0) {
        mprintf("[ERR] second_order step2: kA=%.6f (%.1fppm) kB=%.6f (%.1fppm) implausible\n",
                k_A_new, drift_A, k_B_new, drift_B);
        return;
    }
    if (drift_A > 50.0) mprintf("[WARN] C(%u): k_A drift=%.1f ppm\n", so_a->responder_id, drift_A);
    if (drift_B > 50.0) mprintf("[WARN] C(%u): k_B drift=%.1f ppm\n", so_b->responder_id, drift_B);

    so_a->result_distance_tick = ((double)T_round_AC - k_A_new * (double)T_reply_AC) / 2.0;
    so_b->result_distance_tick = ((double)T_round_BC - k_B_new * (double)T_reply_BC) / 2.0;

    mprintf("[DIST2] A(%u)<->C(%u): %.1f ticks  B(%u)<->C(%u): %.1f ticks  kA=%.6f kB=%.6f\n",
            so_a->initiator_id, so_a->responder_id, so_a->result_distance_tick,
            so_b->initiator_id, so_b->responder_id, so_b->result_distance_tick,
            k_A_new, k_B_new);

    k_ema_update(k_A, k_A_new, 0.20);
    k_ema_update(k_B, k_B_new, 0.20);
}


static void calculate_third_order_distance(
    third_order_t *to, const first_order_t *first,
    double tof_ac_ticks, double tof_bc_ticks,
    double tof_ad_ticks, double tof_bd_ticks,
    double *k_ac, double *k_bc, double *k_ad, double *k_bd)
{
    if (!to || !first) return;

    to->result_distance_tick = -1.0;  /* default until successfully computed */

    const twr_observation_t *obs_c = &to->twr_observation_c;
    const twr_observation_t *obs_d = &to->twr_observation_d;

    if (tof_ac_ticks <= 0.0 || tof_bc_ticks <= 0.0 ||
        tof_ad_ticks <= 0.0 || tof_bd_ticks <= 0.0) {
        mprintf("[ERR] third-order %04X<->%04X: second-order ToFs not ready "
                "AC=%.1f BC=%.1f AD=%.1f BD=%.1f\n",
                to->initiator_id, to->responder_id,
                tof_ac_ticks, tof_bc_ticks, tof_ad_ticks, tof_bd_ticks);
        return;
    }

    if (*k_ac == 1.0 || *k_ad == 1.0 || *k_bc == 1.0 || *k_bd == 1.0) {
        mprintf("[WARN] third-order %04X<->%04X: k not yet estimated\n",
                to->initiator_id, to->responder_id);
        return;
    }

    if (to->twr.answer_rx.ts == 0) {
        mprintf("[ERR] third-order %04X<->%04X: D_rx_C timestamp missing\n",
                to->initiator_id, to->responder_id);
        return;
    }

    const uint64_t poll_tx = first->twr.poll_tx;
    const uint64_t resp_tx = first->twr.resp_tx;

    double c_off_A  = (double)((to->twr.init_tx      - obs_c->poll_rx.ts) & UWB_40BIT_MASK) / *k_ac;
    double d_off_A  = (double)((to->twr.answer_rx.ts  - obs_d->poll_rx.ts) & UWB_40BIT_MASK) / *k_ad;
    double tof_cd_A = ((double)poll_tx + tof_ad_ticks + d_off_A)
                    - ((double)poll_tx + tof_ac_ticks + c_off_A);

    double c_off_B  = (double)((to->twr.init_tx      - obs_c->resp_rx.ts) & UWB_40BIT_MASK) / *k_bc;
    double d_off_B  = (double)((to->twr.answer_rx.ts  - obs_d->resp_rx.ts) & UWB_40BIT_MASK) / *k_bd;
    double tof_cd_B = ((double)resp_tx + tof_bd_ticks + d_off_B)
                    - ((double)resp_tx + tof_bc_ticks + c_off_B);

    bool valid_A = (tof_cd_A >= -200.0);
    bool valid_B = (tof_cd_B >= -200.0);

    if (!valid_A && !valid_B) {
        mprintf("[ERR] third-order %04X<->%04X: both estimates failed A=%.1f B=%.1f\n",
                to->initiator_id, to->responder_id, tof_cd_A, tof_cd_B);
        return;
    }

    #define PDIFF_MAX_DB 40.0
    double diff_c_poll = (double)obs_c->poll_rx.pwr_diff_q8 / 256.0;
    double diff_d_poll = (double)obs_d->poll_rx.pwr_diff_q8 / 256.0;
    double diff_c_resp = (double)obs_c->resp_rx.pwr_diff_q8 / 256.0;
    double diff_d_resp = (double)obs_d->resp_rx.pwr_diff_q8 / 256.0;

    if (diff_c_poll < 0.0) diff_c_poll = 0.0;
    if (diff_d_poll < 0.0) diff_d_poll = 0.0;
    if (diff_c_resp < 0.0) diff_c_resp = 0.0;
    if (diff_d_resp < 0.0) diff_d_resp = 0.0;
    if (diff_c_poll > PDIFF_MAX_DB) diff_c_poll = PDIFF_MAX_DB;
    if (diff_d_poll > PDIFF_MAX_DB) diff_d_poll = PDIFF_MAX_DB;
    if (diff_c_resp > PDIFF_MAX_DB) diff_c_resp = PDIFF_MAX_DB;
    if (diff_d_resp > PDIFF_MAX_DB) diff_d_resp = PDIFF_MAX_DB;
    #undef PDIFF_MAX_DB

    double w_A = 0.0, w_B = 0.0;
    if (valid_A) {
        double wC_A = 1.0 / (1.0 + diff_c_poll);
        double wD_A = 1.0 / (1.0 + diff_d_poll);
        w_A = (wC_A < wD_A) ? wC_A : wD_A;
    }
    if (valid_B) {
        double wC_B = 1.0 / (1.0 + diff_c_resp);
        double wD_B = 1.0 / (1.0 + diff_d_resp);
        w_B = (wC_B < wD_B) ? wC_B : wD_B;
    }

    double tof_cd;
    double w_sum = w_A + w_B;
    if (w_sum > 0.0)
        tof_cd = (tof_cd_A * w_A + tof_cd_B * w_B) / w_sum;
    else
        tof_cd = valid_A ? tof_cd_A : tof_cd_B;

    mprintf("[DIST3] %04X<->%04X: A=%.1f (w=%.3f) B=%.1f (w=%.3f) -> %.1f tks (%.3f m)\n",
            to->initiator_id, to->responder_id,
            tof_cd_A, w_A, tof_cd_B, w_B,
            tof_cd, (float)(tof_cd * METERS_PER_TICK));

    if (tof_cd < 0.0) tof_cd = 0.0;

    to->result_distance_tick = tof_cd;

    if (to->result_distance_tick >= 0.0) {
        uint64_t dt_obs = (obs_c->resp_rx.ts - obs_c->poll_rx.ts) & UWB_40BIT_MASK;
        uint64_t T_round_CD = (to->twr.answer_rx.ts - to->twr.init_tx) & UWB_40BIT_MASK;
        uint64_t T_reply_CD = (to->twr.answer_tx    - to->twr.init_tx) & UWB_40BIT_MASK;

        if (dt_obs > 0 && T_reply_CD > 0) {
            double k_cd_new = (double)(T_round_CD - (uint64_t)(2.0 * to->result_distance_tick))
                            / (double)T_reply_CD;
            double drift = fabs(k_cd_new - 1.0) * 1e6;
            if (drift < 500.0) {
                /* α = 0.05 — third-order is TDOA-derived, smooth aggressively */
                k_ema_update(k_ac, k_cd_new, 0.05);
                k_ema_update(k_bc, k_cd_new, 0.05);
                mprintf("[K3] %04X<->%04X: k_cd=%.6f (%.1f ppm) α=0.05\n",
                        to->initiator_id, to->responder_id, k_cd_new, drift);
            } else {
                mprintf("[WARN] K3 %04X<->%04X: k_cd=%.6f (%.1f ppm) implausible, skipped\n",
                        to->initiator_id, to->responder_id, k_cd_new, drift);
            }
        }
    }
}


uint16_t dist_ticks_to_scale(double ticks)
{
    if (ticks < 0.0 || ticks > DIST_SHARE_DIST_MAX_TICKS)
        return 0xFFFF;
    uint32_t val = (uint32_t)(ticks / DIST_SHARE_TICKS_PER_LSB + 0.5);
    if (val >= 0xFFFF) return 0xFFFE;
    return (uint16_t)val;
}

double dist_scale_to_ticks(uint16_t encoded)
{
    if (encoded == 0xFFFF) return -1.0;
    return encoded * DIST_SHARE_TICKS_PER_LSB;
}

/* clamp a float to an integer range */
static int16_t clampi(float v, int16_t lo, int16_t hi) {
    if (v < (float)lo) return lo;
    if (v > (float)hi) return hi;
    return (int16_t)v;
}

/*
 * timing_offset()
 * Linear penalty/bonus based on SS-TWR turnaround time.
 * Zero at T_ref, positive for faster, negative for slower.
 */
static int16_t timing_offset(float t_gap_us) {
    float delta = -CERTAINTY_KT * (t_gap_us - CERTAINTY_TREF_US);
    return clampi(delta, CERTAINTY_TIMING_MIN, CERTAINTY_TIMING_MAX);
}

/*
 * los_offset()
 * Square-root nonlinear penalty/bonus based on RSSI - FP power gap.
 * Zero at DELTA0_DB, positive below (good LOS), negative above (NLOS).
 * Steep in the first ~10 dB above average, flattens beyond that.
 */
static int16_t los_offset(int16_t pwr_diff_q8) {
    int16_t diff_q8 = pwr_diff_q8 - CERTAINTY_DELTA0_Q8;
    float   sign    = (diff_q8 >= 0) ? 1.0f : -1.0f;
    float   delta   = -CERTAINTY_KN_Q8 * sign * sqrtf((float)abs(diff_q8));
    return clampi(delta, CERTAINTY_LOS_MIN, CERTAINTY_LOS_MAX);
}

/* ------------------------------------------------------------------ */

uint8_t compute_certainty(MeasurementType type,
                          float           t_gap_us,
                          int16_t         pwr_diff_q8) {
    if (type == MEAS_NONE) return 0;

    int16_t base, dt, dlos;

    dlos = los_offset(pwr_diff_q8);

    switch (type) {
        case MEAS_DSTWR:
            base = CERTAINTY_BASE_DSTWR;
            dt   = 0;
            break;
        case MEAS_SSTWR:
            base = CERTAINTY_BASE_SSTWR;
            dt   = timing_offset(t_gap_us);
            break;
        case MEAS_TDOA_SSTWR:
            base = CERTAINTY_BASE_TDOA;
            dt   = timing_offset(t_gap_us);
            break;
        default:
            return 0;
    }

    int16_t score = base + dt + dlos;
    if (score < 1)   return 1;
    if (score > 254) return 254;
    return (uint8_t)score;
}

void distance_calculate(uwb_etwr_result_t result)
{
    uint32_t t_start = osKernelGetTickCount();

    if (result == UWB_TWR_RECEIVED) {
        
        /* Build timestamp structures for this round */
        populate_computation_structs(&timestamps);

        /* ── 1. First order ───────────────────────────────────────────── */

        calculate_first_order_distance_ticks(&timestamps.first);

        uint16_t id_A = timestamps.first.initiator_id;
        uint16_t id_B = timestamps.first.responder_id;
        double   tof_ab = timestamps.first.result_distance_tick;

        network_set_distance(id_A, id_B, dist_ticks_to_scale(tof_ab));
        network_set_distance(id_B, id_A, dist_ticks_to_scale(tof_ab));

        int16_t avg_pwr_ab = (timestamps.first.twr.poll_rx.pwr_diff_q8
                            + timestamps.first.twr.resp_rx.pwr_diff_q8
                            + timestamps.first.twr.final_rx.pwr_diff_q8) / 3;
        uint8_t cert_ab = compute_certainty(MEAS_DSTWR, 0, avg_pwr_ab);
        network_update_certainty(id_A, id_B, cert_ab);
        network_update_certainty(id_B, id_A, cert_ab);
        mprintf("[TWR] 0x%04X<->0x%04X: %.3f m\n", id_A, id_B, tof_ab * METERS_PER_TICK);

        if (tof_ab < 0.0) goto done;

        /* ── 2. Second order ──────────────────────────────────────────── */
        uint16_t processed_C[MAX_PASSIVE];
        uint8_t  processed_count = 0;

        for (uint8_t i = 0; i < timestamps.second_count; i++) {
            second_order_t *cur = &timestamps.second[i];
            if (cur->initiator_id != id_A) continue;
            uint16_t id_C = cur->responder_id;

            bool already_done = false;
            for (uint8_t p = 0; p < processed_count; p++)
                if (processed_C[p] == id_C) { already_done = true; break; }
            if (already_done) continue;

            second_order_t *entry_ac = cur, *entry_bc = NULL;
            for (uint8_t j = 0; j < timestamps.second_count; j++) {
                if (timestamps.second[j].initiator_id == id_B &&
                    timestamps.second[j].responder_id == id_C) {
                    entry_bc = &timestamps.second[j]; break;
                }
            }
            if (!entry_bc) { mprintf("[ERR] second-order: no B-C for C=0x%04X\n", id_C); continue; }

            double k_A = network_get_k(id_A, id_C);
            double k_B = network_get_k(id_B, id_C);

            double prev_ac = dist_scale_to_ticks(network_get_distance(id_A, id_C));
            double prev_bc = dist_scale_to_ticks(network_get_distance(id_B, id_C));

            calculate_second_order_distances(entry_ac, entry_bc,
                                            &timestamps.first,
                                            prev_ac, prev_bc, &k_A, &k_B);

            network_set_distance(id_A, id_C, dist_ticks_to_scale(entry_ac->result_distance_tick));
            network_set_distance(id_C, id_A, dist_ticks_to_scale(entry_ac->result_distance_tick));
            network_set_distance(id_B, id_C, dist_ticks_to_scale(entry_bc->result_distance_tick));
            network_set_distance(id_C, id_B, dist_ticks_to_scale(entry_bc->result_distance_tick));

            int16_t avg_pwr_bc = (entry_bc->twr.init_rx.pwr_diff_q8
                                + entry_bc->twr.answer_rx.pwr_diff_q8) / 2;
            uint64_t gap_bc = (entry_bc->twr.answer_rx.ts - entry_bc->twr.init_tx) & UWB_40BIT_MASK;
            network_update_certainty(id_B, id_C,
                compute_certainty(MEAS_SSTWR, (float)gap_bc * DWT_TICK_TO_US, avg_pwr_bc));
            network_update_certainty(id_C, id_B,
                compute_certainty(MEAS_SSTWR, (float)gap_bc * DWT_TICK_TO_US, avg_pwr_bc));

            int16_t avg_pwr_ac = (entry_ac->twr.init_rx.pwr_diff_q8
                                + entry_ac->twr.answer_rx.pwr_diff_q8) / 2;
            uint64_t gap_ac = (entry_ac->twr.answer_rx.ts - entry_ac->twr.init_tx) & UWB_40BIT_MASK;
            network_update_certainty(id_A, id_C,
                compute_certainty(MEAS_SSTWR, (float)gap_ac * DWT_TICK_TO_US, avg_pwr_ac));
            network_update_certainty(id_C, id_A,
                compute_certainty(MEAS_SSTWR, (float)gap_ac * DWT_TICK_TO_US, avg_pwr_ac));

            network_set_k(id_A, id_C, k_A);
            network_set_k(id_B, id_C, k_B);
            processed_C[processed_count++] = id_C;
        }

         /* ── 3. Third order ───────────────────────────────────────────── */
        for (uint8_t t = 0; t < timestamps.third_count; t++) {
            third_order_t *to = &timestamps.third[t];
            uint16_t id_C3 = to->initiator_id;
            uint16_t id_D  = to->responder_id;

            /* Find second-order entries to get tof_ac, tof_bc, tof_ad, tof_bd */
            double tof_ac = -1.0, tof_bc = -1.0, tof_ad = -1.0, tof_bd = -1.0;
            for (uint8_t i = 0; i < timestamps.second_count; i++) {
                const second_order_t *s = &timestamps.second[i];
                if (s->initiator_id == id_A && s->responder_id == id_C3) tof_ac = s->result_distance_tick;
                if (s->initiator_id == id_B && s->responder_id == id_C3) tof_bc = s->result_distance_tick;
                if (s->initiator_id == id_A && s->responder_id == id_D)  tof_ad = s->result_distance_tick;
                if (s->initiator_id == id_B && s->responder_id == id_D)  tof_bd = s->result_distance_tick;
            }

            double k_ac = network_get_k(id_A, id_C3);
            double k_bc = network_get_k(id_B, id_C3);
            double k_ad = network_get_k(id_A, id_D);
            double k_bd = network_get_k(id_B, id_D);

            calculate_third_order_distance(to, &timestamps.first,
                                            tof_ac, tof_bc, tof_ad, tof_bd,
                                            &k_ac, &k_bc, &k_ad, &k_bd);

            network_set_k(id_A, id_C3, k_ac);
            network_set_k(id_B, id_C3, k_bc);
            network_set_k(id_A, id_D,  k_ad);
            network_set_k(id_B, id_D,  k_bd);

            network_set_distance(id_C3, id_D, dist_ticks_to_scale(to->result_distance_tick));
            network_set_distance(id_D, id_C3, dist_ticks_to_scale(to->result_distance_tick));

            int16_t worst_pwr_A = (to->twr_observation_c.poll_rx.pwr_diff_q8
                                + to->twr_observation_d.poll_rx.pwr_diff_q8) / 2;
            int16_t worst_pwr_B = (to->twr_observation_c.resp_rx.pwr_diff_q8
                                + to->twr_observation_d.resp_rx.pwr_diff_q8) / 2;
            int16_t avg_pwr_cd  = (worst_pwr_A + worst_pwr_B) / 2;
            uint64_t gap_cd = (to->twr.answer_rx.ts - to->twr.init_tx) & UWB_40BIT_MASK;
            uint8_t cert_cd = compute_certainty(MEAS_TDOA_SSTWR,
                                                (float)gap_cd * DWT_TICK_TO_US, avg_pwr_cd);
            network_update_certainty(id_C3, id_D, cert_cd);
            network_update_certainty(id_D, id_C3, cert_cd);
        }
        done:;
    }
    /* Pad to exactly 10 ms from entry regardless of which path was taken */
    uint32_t elapsed = osKernelGetTickCount() - t_start;
    if (elapsed < 10U) osDelay(10U - elapsed);
}
