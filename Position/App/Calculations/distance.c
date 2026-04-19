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
#define UWB_40BIT_MASK 0xFFFFFFFFFFULL
//TODO encoding horiyontal velocity can lose the +/- sign and thus incerease the accuracy, as we stil dont know the direction
//we cannot do it for vertical, as we know if its up or down.

static inline uint8_t vel_ms_to_u8(float vel_ms)
{
    float clamped = vel_ms < -VEL_SHARE_RANGE_MS ? -VEL_SHARE_RANGE_MS
                  : vel_ms >  VEL_SHARE_RANGE_MS ?  VEL_SHARE_RANGE_MS
                  : vel_ms;
    return (uint8_t)((clamped + VEL_SHARE_RANGE_MS) / VEL_SHARE_MS_PER_LSB + 0.5f);
}

static inline float vel_u8_to_ms(uint8_t encoded)
{
    return encoded * VEL_SHARE_MS_PER_LSB - VEL_SHARE_RANGE_MS;
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

    net->self.imu_vel_vert  = vel_ms_to_u8(vel_z);        /* float m/s → uint8_t */
    net->self.imu_vel_horiz = vel_ms_to_u8(speed_horiz);  /* float m/s → uint8_t */

    node_t *init_node = find_peer(initiator_id);
    if (init_node) {
        init_node->imu_vel_vert  = vel_ms_to_u8(meas->final.IMU_vel_vert);
        init_node->imu_vel_horiz = vel_ms_to_u8(meas->final.IMU_vel_horiz);
    }

    /* -----------------------------------------------------------------------
     * 1. FIRST ORDER: Initiator (A) and Responder (Self/B)
     *    vel fields in timestamps_t are float — decode uint8_t from wire back
     *    to float for computation, keep own IMU as raw float.
     * ----------------------------------------------------------------------- */
    ts->first.initiator_id        = initiator_id;
    ts->first.responder_id        = responder_id;
    ts->first.init_vel_horiz      = meas->final.IMU_vel_horiz;  /* already float from msg */
    ts->first.init_vel_vert       = meas->final.IMU_vel_vert;
    ts->first.responder_vel_horiz = speed_horiz;                /* raw float, not re-encoded */
    ts->first.responder_vel_vert  = vel_z;

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
            passive_node->imu_vel_vert  = vel_ms_to_u8(meas->passive[i].IMU_vel_vert);
            passive_node->imu_vel_horiz = vel_ms_to_u8(meas->passive[i].IMU_vel_horiz);
        }

        if ((int)ts->second_count + 2 > MAX_SECOND_ORDER) { 
            mprintf("[ERR] populate: second_order overflow at passive %u\n", i);
            break;
        }

        /* A <-> Passive */
        second_order_t *so_a      = &ts->second[ts->second_count++];
        so_a->initiator_id        = initiator_id;
        so_a->responder_id        = passive_id;
        so_a->init_vel_horiz      = meas->final.IMU_vel_horiz;        /* float from msg */
        so_a->init_vel_vert       = meas->final.IMU_vel_vert;
        so_a->responder_vel_horiz = meas->passive[i].IMU_vel_horiz;   /* float from msg */
        so_a->responder_vel_vert  = meas->passive[i].IMU_vel_vert;
        so_a->twr_observation     = shared_obs;
        so_a->twr.init_tx         = meas->final.poll_tx_ts;
        so_a->twr.init_rx         = shared_obs.poll_rx;
        so_a->twr.answer_tx       = meas->passive[i].passive_tx_ts;
        so_a->twr.answer_rx       = a_rx_of_passive;

        /* B <-> Passive */
        second_order_t *so_b      = &ts->second[ts->second_count++];
        so_b->initiator_id        = responder_id;
        so_b->responder_id        = passive_id;
        so_b->init_vel_horiz      = speed_horiz;                       /* raw float */
        so_b->init_vel_vert       = vel_z;
        so_b->responder_vel_horiz = meas->passive[i].IMU_vel_horiz;   /* float from msg */
        so_b->responder_vel_vert  = meas->passive[i].IMU_vel_vert;
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
        to->initiator_id        = meas->passive_device_id[i];
        to->responder_id        = meas->passive_device_id[j];
        to->init_vel_horiz      = meas->passive[i].IMU_vel_horiz;   /* float from msg */
        to->init_vel_vert       = meas->passive[i].IMU_vel_vert;
        to->responder_vel_horiz = meas->passive[j].IMU_vel_horiz;   /* float from msg */
        to->responder_vel_vert  = meas->passive[j].IMU_vel_vert;

        uwb_rx_meas_t j_rx_of_i = {0};
        for (uint8_t k = 0; k < meas->passive[j].entry_count; k++) {
            if (meas->passive[j].entry_ids[k] == meas->passive_device_id[i]) {
                j_rx_of_i.ts      = meas->passive[j].entries[k];
                j_rx_of_i.pwr_diff_q8 = meas->passive[j].entry_pwr_diff_q8[k];
                break;
            }
        }

        uwb_rx_meas_t i_rx_of_j = {0};
        for (uint8_t k = 0; k < meas->passive[i].entry_count; k++) {
            if (meas->passive[i].entry_ids[k] == meas->passive_device_id[j]) {
                i_rx_of_j.ts      = meas->passive[i].entries[k];
                i_rx_of_j.pwr_diff_q8 = meas->passive[i].entry_pwr_diff_q8[k];
                break;
            }
        }

        to->twr_observation.poll_rx = j_rx_of_i;
        to->twr_observation.resp_rx = i_rx_of_j;
        to->twr.init_tx             = meas->passive[i].passive_tx_ts;
        to->twr.init_rx             = j_rx_of_i;
        to->twr.answer_tx           = meas->passive[j].passive_tx_ts;
        to->twr.answer_rx           = i_rx_of_j;
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

double calculate_first_order_distance_ticks(const first_order_t *f)
{
    if (!f) return -1.0;

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
        return -1.0;
    }

    /* 2. Same-domain constraint: both on the responder's clock */
    int64_t diff1 = (int64_t)T_round1 - (int64_t)T_reply1;
    if (diff1 < -500) {
        mprintf("[ERR] first_order: T_reply1(%u) massively exceeds T_round1(%u)\n",
                (uint32_t)T_reply1, (uint32_t)T_round1);
        return -1.0;
    } else if (diff1 < 0) {
        T_round1 = T_reply1; /* Clamp to 0 distance for this segment */
    }

    int64_t diff2 = (int64_t)T_round2 - (int64_t)T_reply2;
    if (diff2 < -500) {
        mprintf("[ERR] first_order: T_reply2 massively exceeds T_round2\n");
        return -1.0;
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
        return -1.0;
    }

    double tof_ticks = (dR1 * dR2 - dP1 * dP2) / den;

    /* 4. Sanity — reject catastrophic cancellation and physically impossible values */
    if (tof_ticks < -200.0) {
        mprintf("[ERR] first_order: tof=%.1f ticks — catastrophic cancellation, check final_tx latch\n",
                tof_ticks);
        return -1.0;
    }
/*
    mprintf("[DIST1] A(%u)<->B(%u): %.1f ticks\n",
            f->initiator_id, f->responder_id, tof_ticks);
*/
    return tof_ticks;
}


second_order_result_t calculate_second_order_distances(
    const second_order_t *so_a,
    const second_order_t *so_b,
    const first_order_t  *first,
    double                tof_ab_ticks,
    double                tof_ac_init,
    double                tof_bc_init,
    double               *k_A,
    double               *k_B)
{
    second_order_result_t result = { -1.0, -1.0 };
    if (!so_a || !so_b || !first || !k_A || !k_B) return result;

    if (so_a->responder_id != so_b->responder_id) {
        mprintf("[ERR] second_order: passive ID mismatch (%u vs %u)\n",
                so_a->responder_id, so_b->responder_id);
        return result;
    }
    if (so_a->initiator_id != first->initiator_id ||
        so_b->initiator_id != first->responder_id) {
        mprintf("[ERR] second_order: initiator IDs don't match first-order — entries swapped?\n");
        return result;
    }

    /* ── shared references ─────────────────────────────── */
    const twr_timestamps_t  *ft  = &first->twr;
    const twr_observation_t *obs = &so_a->twr_observation;

    /* ── passive C observation window ──────────────────── */
    uint64_t dt_C = (obs->resp_rx.ts - obs->poll_rx.ts) & UWB_40BIT_MASK;
    if (dt_C == 0) { mprintf("[ERR] second_order: dt_C == 0\n"); return result; }

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
        return result;
    } else if (tof_ab_ticks >= T_round1_A) {
        tof_ab_ticks = T_round1_A - 0.1; /* Clamp to keep dt_A_ref slightly positive */
        dt_A_ref = 0.1;
    }

    if (dt_B_ref < -500.0) {
        mprintf("[ERR] second_order: massively negative dt_B_ref\n");
        return result;
    } else if (dt_B_ref <= 0.0) {
        dt_B_ref = 0.1; /* Clamp to keep dt_B_ref slightly positive */
    }

    /* ═══════════════════════════════════════════════════
     * STEP 1 — correction source
     * Use prior estimates when available; fall back to
     * the geometric approximation on the first cycle.
     * ═══════════════════════════════════════════════════ */
    const bool have_prior = (tof_ac_init > 0.0 && tof_bc_init > 0.0);
    double correction_s1;

    if (have_prior) {
        correction_s1 = tof_bc_init - tof_ac_init;
        mprintf("[STEP1] A(%u)<->C(%u): %.1f ticks  B(%u)<->C(%u): %.1f ticks  (prior)\n",
                so_a->initiator_id, so_a->responder_id, tof_ac_init,
                so_b->initiator_id, so_b->responder_id, tof_bc_init);
    } else {
        double k_A_s1    = dt_A_ref / (double)dt_C;
        double k_B_s1    = dt_B_ref / (double)dt_C;
        double tof_ac_s1 = ((double)T_round_AC - k_A_s1 * (double)T_reply_AC) / 2.0;
        double tof_bc_s1 = ((double)T_round_BC - k_B_s1 * (double)T_reply_BC) / 2.0;

        mprintf("[STEP1] A(%u)<->C(%u): %.1f ticks  B(%u)<->C(%u): %.1f ticks  (geometric)\n",
                so_a->initiator_id, so_a->responder_id, tof_ac_s1,
                so_b->initiator_id, so_b->responder_id, tof_bc_s1);

        if (tof_ac_s1 < -200.0 || tof_bc_s1 < -200.0) {
            mprintf("[ERR] second_order: step1 ToF implausibly negative (ac=%.1f bc=%.1f)\n",
                    tof_ac_s1, tof_bc_s1);
            return result;
        }
        correction_s1 = tof_bc_s1 - tof_ac_s1;
    }

    /* ═══════════════════════════════════════════════════
     * STEP 2 — corrected k using best available correction
     * ═══════════════════════════════════════════════════ */
    double dt_A_ref_corr = dt_A_ref + correction_s1;
    double dt_B_ref_corr = dt_B_ref + tof_ab_ticks + correction_s1;

    if (dt_A_ref_corr <= 0.0 || dt_B_ref_corr <= 0.0) {
        mprintf("[WARN] second_order: corrected reference <= 0 — extreme geometry\n");
        if (have_prior) {
            /* Fall back to geometric step-1 */
            double k_A_s1 = dt_A_ref / (double)dt_C;
            double k_B_s1 = dt_B_ref / (double)dt_C;
            result.tof_ac_ticks = ((double)T_round_AC - k_A_s1 * (double)T_reply_AC) / 2.0;
            result.tof_bc_ticks = ((double)T_round_BC - k_B_s1 * (double)T_reply_BC) / 2.0;
        }
        return result;
    }

    double k_A_new = dt_A_ref_corr / (double)dt_C;
    double k_B_new = dt_B_ref_corr / (double)dt_C;

    double drift_A = fabs(k_A_new - 1.0) * 1e6;
    double drift_B = fabs(k_B_new - 1.0) * 1e6;
    if (drift_A > 500.0 || drift_B > 500.0) {
        mprintf("[ERR] second_order step2: kA=%.6f (%.1fppm) kB=%.6f (%.1fppm) implausible\n",
                k_A_new, drift_A, k_B_new, drift_B);
        return result;
    }
    if (drift_A > 50.0) mprintf("[WARN] C(%u): k_A drift=%.1f ppm\n", so_a->responder_id, drift_A);
    if (drift_B > 50.0) mprintf("[WARN] C(%u): k_B drift=%.1f ppm\n", so_b->responder_id, drift_B);

    result.tof_ac_ticks = (float)(((double)T_round_AC - k_A_new * (double)T_reply_AC) / 2.0);
    result.tof_bc_ticks = (float)(((double)T_round_BC - k_B_new * (double)T_reply_BC) / 2.0);

    /* ═══════════════════════════════════════════════════
     * Smooth k values back to caller (EMA, α = 0.2).
     * Seed directly on first call (*k_A == *k_B == 1.0).
     * ═══════════════════════════════════════════════════ */
    #define K_SMOOTH_ALPHA 0.2
        if (*k_A == 1.0 && *k_B == 1.0) {
            *k_A = k_A_new;
            *k_B = k_B_new;
        } else {
            *k_A = (1.0 - K_SMOOTH_ALPHA) * (*k_A) + K_SMOOTH_ALPHA * k_A_new;
            *k_B = (1.0 - K_SMOOTH_ALPHA) * (*k_B) + K_SMOOTH_ALPHA * k_B_new;
        }
    #undef K_SMOOTH_ALPHA

    /*mprintf("[DIST2] A(%u)<->C(%u): %.1f ticks  B(%u)<->C(%u): %.1f ticks  (kA=%.6f kB=%.6f)\n",
            so_a->initiator_id, so_a->responder_id, result.tof_ac_ticks,
            so_b->initiator_id, so_b->responder_id, result.tof_bc_ticks,
            *k_A, *k_B);

    mprintf("[RAW] poll_tx=%u poll_rx=%u resp_tx=%u resp_rx=%u final_tx=%u final_rx=%u\n",
            (uint32_t)ft->poll_tx,      (uint32_t)ft->poll_rx.ts,
            (uint32_t)ft->resp_tx,      (uint32_t)ft->resp_rx.ts,
            (uint32_t)ft->final_tx,     (uint32_t)ft->final_rx.ts);*/

    return result;
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

        /* First-order DS-TWR: A <-> B (A = initiator, B = self) */
        double first_order_distance = calculate_first_order_distance_ticks(&timestamps.first);
        mprintf("[CERT] self=0x%04X, initiator=0x%04X, responder=0x%04X\n", network_get_ownid(), timestamps.first.initiator_id, timestamps.first.responder_id);

        uint16_t id_A = timestamps.first.initiator_id;
        uint16_t id_B = timestamps.first.responder_id;  /* self */

        /* Store A <-> B distance symmetrically */
        network_set_distance(id_A, id_B, dist_ticks_to_scale(first_order_distance));
        network_set_distance(id_B, id_A, dist_ticks_to_scale(first_order_distance));

        /*
         * DS-TWR certainty: +3 for the directly ranged pair A <-> B.
         * This always runs, even with no passive nodes.
         */
        uint16_t avg_pwr_diff_q8 = (timestamps.first.twr.poll_rx.pwr_diff_q8 + timestamps.first.twr.resp_rx.pwr_diff_q8 + 
                                        timestamps.first.twr.final_rx.pwr_diff_q8) / 3;
        uint8_t certainty_ab = compute_certainty(MEAS_DSTWR, 0, avg_pwr_diff_q8);
        network_update_certainty(id_A, id_B, certainty_ab);
        network_update_certainty(id_B, id_A, certainty_ab);

        mprintf("---After DS-TWR -----");
        network_print_certainty();

        mprintf("[TWR] 0x%04X<->0x%04X: %.3f m\n",
                id_A,
                id_B,
                first_order_distance * METERS_PER_TICK);

        /* Second-order: A/B <-> C via passive(s) */
        uint16_t processed_C[NETWORK_MAX_PEERS];
        uint8_t  processed_count = 0;

        for (uint8_t i = 0; i < timestamps.second_count; i++) {

            second_order_t *cur = &timestamps.second[i];

            /* C is always the responder; skip if this is not an A->C entry */
            if (cur->initiator_id != id_A) continue;
            uint16_t id_C = cur->responder_id;

            /* Skip already processed C */
            bool already_done = false;
            for (uint8_t p = 0; p < processed_count; p++) {
                if (processed_C[p] == id_C) {
                    already_done = true;
                    break;
                }
            }
            if (already_done) continue;

            /* Find the matching B->C entry */
            second_order_t *entry_ac = cur;
            second_order_t *entry_bc = NULL;
            for (uint8_t j = 0; j < timestamps.second_count; j++) {
                if (timestamps.second[j].initiator_id == id_B &&
                    timestamps.second[j].responder_id == id_C) {
                    entry_bc = &timestamps.second[j];
                    break;
                }
            }

            if (entry_bc == NULL) {
                mprintf("[ERR] second-order: no B->C entry for C=0x%04X\n", id_C);
                continue;
            }

            double k_A     = network_get_k(id_A, id_C);
            double k_B     = network_get_k(id_B, id_C);
            double prev_ac = dist_scale_to_ticks(network_get_distance(id_A, id_C));
            double prev_bc = dist_scale_to_ticks(network_get_distance(id_B, id_C));

            second_order_result_t res = calculate_second_order_distances(
                    entry_ac, entry_bc,
                    &timestamps.first,
                    first_order_distance,
                    prev_ac, prev_bc,
                    &k_A, &k_B);

            if (res.tof_ac_ticks < 0.0) res.tof_ac_ticks = 0.0;
            if (res.tof_bc_ticks < 0.0) res.tof_bc_ticks = 0.0;

            network_set_distance(id_A, id_C, dist_ticks_to_scale(res.tof_ac_ticks));
            network_set_distance(id_C, id_A, dist_ticks_to_scale(res.tof_ac_ticks));

            network_set_distance(id_B, id_C, dist_ticks_to_scale(res.tof_bc_ticks));
            network_set_distance(id_C, id_B, dist_ticks_to_scale(res.tof_bc_ticks));

            /*
             * SS-TWR certainty: +1 for self <-> passive (C).
             * id_A already received DS-TWR +3 above.
             * Do NOT update id_A <-> id_C here — self has no direct
             * measurement of that pair.
             */
            uint16_t avg_pwr_diff_q8_bc = (entry_bc->twr.init_rx.pwr_diff_q8 + entry_bc->twr.answer_rx.pwr_diff_q8) / 2;
            uint8_t certainty_bc = compute_certainty(MEAS_SSTWR, 
                (float)(entry_bc->twr.answer_rx.ts - entry_bc->twr.init_tx) *DWT_TICK_TO_US, avg_pwr_diff_q8_bc);
            network_update_certainty(id_B, id_C, certainty_bc);
            network_update_certainty(id_C, id_B, certainty_bc);

            uint16_t avg_pwr_diff_q8_ac = (entry_ac->twr.init_rx.pwr_diff_q8 + entry_ac->twr.answer_rx.pwr_diff_q8) / 2;
            uint8_t certainty_ac = compute_certainty(MEAS_SSTWR, 
                (float)(entry_ac->twr.answer_rx.ts - entry_ac->twr.init_tx) *DWT_TICK_TO_US, avg_pwr_diff_q8_ac);
            network_update_certainty(id_A, id_C, certainty_ac);
            network_update_certainty(id_C, id_A, certainty_ac);

            mprintf("---After SS-TWR -----");
            network_print_certainty();

            /* Update k smoothing for this passive */
            network_set_k(id_A, id_C, k_A);
            network_set_k(id_B, id_C, k_B);

            processed_C[processed_count++] = id_C;
        }
    }

    /* Pad to exactly 10 ms from entry regardless of which path was taken */
    uint32_t elapsed = osKernelGetTickCount() - t_start;
    if (elapsed < 10U) osDelay(10U - elapsed);
}

/*
uint64_t position_calibrate_timestamp(uint64_t orig_timestamp){
    return orig_timestamp-1;
}
    */