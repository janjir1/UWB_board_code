#include "cmsis_os.h"
#include "cmsis_os2.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "../Generic/my_print.h"
#include "DWM3000_setup.h"
#include "DWM3000_driver.h"
#include "messages.h"
#include "uwb_network.h"
#include "uwb_exchange.h"
#include "position.h"
#include "lsm6dsv.h"

timestamps_t timestamps;

void populate_computation_structs(timestamps_t *ts) {
    if (!ts) return;

    // Retrieve the global network state
    network_t *net = network_get_network();
    if (!net) return; 

    // Retrieve peers to find the initiator (always at index 0)
    uint8_t peer_count = 0;
    const node_t *peers = network_get_peers(&peer_count);
    
    // Safety check: Ensure we actually have an initiator in the peer list
    if (!peers || peer_count == 0) return;
    
    uint16_t initiator_id = peers[0].id;

    // Self is always the responder in the primary exchange
    uint16_t responder_id = network_get_ownid(); 

    // 0. Zero out the structure
    memset(ts, 0, sizeof(timestamps_t));
    const measurements_t *meas = &net->measurements;

    uint32_t flags = osThreadFlagsWait(0x02, osFlagsWaitAll, 2);
    float pitch_rad = 0, speed_horiz = 0, vel_z = 0;
    if (flags == 0x02)
        imu_get_results(&pitch_rad, &speed_horiz, &vel_z);

    /* -----------------------------------------------------------------------
    * 1. FIRST ORDER: Initiator (A) and Responder (Self/B)
    * ----------------------------------------------------------------------- */

    ts->first.initiator_id = initiator_id;
    ts->first.responder_id = responder_id;

    // A's velocity — embedded in FINAL message
    ts->first.init_vel_horiz      = meas->final.IMU_vel_horiz;
    ts->first.init_vel_vert       = meas->final.IMU_vel_vert;

    // Self (B)'s velocity — local IMU
    ts->first.responder_vel_horiz = speed_horiz;
    ts->first.responder_vel_vert  = vel_z;

    // Local hardware timestamps (Self is Responder: RX Poll, TX Resp, RX Final)
    ts->first.twr.poll_rx  = meas->poll_rx;
    ts->first.twr.resp_tx  = meas->resp_tx;
    ts->first.twr.final_rx = meas->final_rx;

    // Timestamps embedded in A's FINAL message
    ts->first.twr.poll_tx         = meas->final.poll_tx_ts;
    ts->first.twr.resp_rx.ts      = meas->final.resp_rx_ts;
    ts->first.twr.resp_rx.rssi_q8 = meas->final.resp_rssi_q8;
    ts->first.twr.final_tx        = meas->final.final_tx_ts;


    /* -----------------------------------------------------------------------
    * 2. SECOND ORDER: one pair per passive toward A, one toward B
    *    Both pairs share the same twr_observation (passive saw POLL + RESP)
    * ----------------------------------------------------------------------- */
    uint8_t p_count = meas->passive_count;
    for (uint8_t i = 0; i < p_count; i++) {
        uint16_t passive_id = meas->passive_device_id[i];

        // Shared observation: passive heard both A's POLL and B's RESP
        twr_observation_t shared_obs = {
            .poll_rx = {
                .ts      = meas->passive[i].poll_rx_ts,
                .rssi_q8 = meas->passive[i].poll_rssi_q8,
            },
            .resp_rx = {
                .ts      = meas->passive[i].resp_rx_ts,
                .rssi_q8 = meas->passive[i].resp_rssi_q8,
            },
        };

        // A's RX of this passive's frame (embedded in FINAL payload)
        uwb_rx_meas_t a_rx_of_passive = {0};
        for (uint8_t k = 0; k < meas->final.entry_count; k++) {
            if (meas->final.entry_id[k] == passive_id) {
                a_rx_of_passive.ts      = meas->final.entries[k];
                a_rx_of_passive.rssi_q8 = meas->final.entry_rssi_q8[k];
                break;
            }
        }

        // --- A <-> Passive (e.g. A-C, A-D, A-E) ---
        second_order_t *so_a       = &ts->second[ts->second_count++];
        so_a->initiator_id         = initiator_id;
        so_a->responder_id         = passive_id;
        so_a->init_vel_horiz       = meas->final.IMU_vel_horiz;
        so_a->init_vel_vert        = meas->final.IMU_vel_vert;
        so_a->responder_vel_horiz  = meas->passive[i].IMU_vel_horiz;
        so_a->responder_vel_vert   = meas->passive[i].IMU_vel_vert;
        so_a->twr_observation      = shared_obs;          // passive saw POLL + RESP
        so_a->twr.init_tx          = meas->final.poll_tx_ts;   // A sent POLL
        so_a->twr.init_rx          = shared_obs.poll_rx;       // passive received POLL
        so_a->twr.answer_tx        = meas->passive[i].passive_tx_ts;
        so_a->twr.answer_rx        = a_rx_of_passive;          // A received PASSIVE (from FINAL)

        // --- B <-> Passive (e.g. B-C, B-D, B-E) ---
        second_order_t *so_b       = &ts->second[ts->second_count++];
        so_b->initiator_id         = responder_id;             // Self = B
        so_b->responder_id         = passive_id;
        so_b->init_vel_horiz       = speed_horiz;
        so_b->init_vel_vert        = vel_z; 
        so_b->responder_vel_horiz  = meas->passive[i].IMU_vel_horiz;
        so_b->responder_vel_vert   = meas->passive[i].IMU_vel_vert;
        so_b->twr_observation      = shared_obs;          // same — passive saw POLL + RESP
        so_b->twr.init_tx          = meas->resp_tx;            // B sent RESP
        so_b->twr.init_rx          = shared_obs.resp_rx;       // passive received RESP
        so_b->twr.answer_tx        = meas->passive[i].passive_tx_ts;
        so_b->twr.answer_rx        = meas->passive_rx[i];      // Self (B) received PASSIVE
    }

    /* -----------------------------------------------------------------------
     * 3. THIRD ORDER: Passive-to-Passive combinations
     * ----------------------------------------------------------------------- */
    
    for (uint8_t i = 0; i < p_count; i++) {
    for (uint8_t j = i + 1; j < p_count; j++) {
        third_order_t *to = &ts->third[ts->third_count++];

        to->initiator_id    = meas->passive_device_id[i];
        to->responder_id    = meas->passive_device_id[j];
        to->init_vel_horiz  = meas->passive[i].IMU_vel_horiz;
        to->init_vel_vert   = meas->passive[i].IMU_vel_vert;
        to->responder_vel_horiz = meas->passive[j].IMU_vel_horiz;
        to->responder_vel_vert  = meas->passive[j].IMU_vel_vert;

        // passive[j]'s RX of passive[i]'s frame
        uwb_rx_meas_t j_rx_of_i = {0};
        for (uint8_t k = 0; k < meas->passive[j].entry_count; k++) {
            if (meas->passive[j].entry_ids[k] == meas->passive_device_id[i]) {
                j_rx_of_i.ts      = meas->passive[j].entries[k];
                j_rx_of_i.rssi_q8 = meas->passive[j].entry_rssi_q8[k];
                break;
            }
        }

        // passive[i]'s RX of passive[j]'s frame  ← new, symmetric search
        uwb_rx_meas_t i_rx_of_j = {0};
        for (uint8_t k = 0; k < meas->passive[i].entry_count; k++) {
            if (meas->passive[i].entry_ids[k] == meas->passive_device_id[j]) {
                i_rx_of_j.ts      = meas->passive[i].entries[k];
                i_rx_of_j.rssi_q8 = meas->passive[i].entry_rssi_q8[k];
                break;
            }
        }

        // ✅ twr_observation: mutual passive observations
        to->twr_observation.poll_rx = j_rx_of_i;   // [j] saw [i]'s PASSIVE
        to->twr_observation.resp_rx = i_rx_of_j;   // [i] saw [j]'s PASSIVE

        // SS-TWR
        to->twr.init_tx   = meas->passive[i].passive_tx_ts;
        to->twr.init_rx   = j_rx_of_i;                // [j] received [i]
        to->twr.answer_tx = meas->passive[j].passive_tx_ts;
        to->twr.answer_rx = i_rx_of_j;                // ✅ [i] received [j]
    }
}
}

/* Since populate() always calls memset(ts, 0, ...) first,
 * any field still zero after populate() was never written. */

static bool check_uwb_rx_meas(const uwb_rx_meas_t *m, const char *name)
{
    if (m->ts == 0) {
        /* memset guarantees: ts==0 means populate() never touched this field */
        mprintf("[FAIL] %s.ts == 0 (unpopulated)\n", name);
        return false;
    }
    if (m->rssi_q8 == 0) {
        /* 0 dBm is physically possible, but suspicious — warn only */
        mprintf("[WARN] %s.rssi_q8 == 0 (0 dBm — check if intentional)\n", name);
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

void distance_calculate(uwb_etwr_result_t result)
{
    uint32_t t_start = osKernelGetTickCount();
    

    if (result == UWB_TWR_RECEIVED) {
            
            populate_computation_structs(&timestamps);
            bool result = test_timestamps_populated(&timestamps);
            if (result) {
                mprintf("Timestamps population test PASSED!\r\n");
            } else {
                mprintf("Timestamps population test FAILED!\r\n");
            }
            // TODO: use real function
            network_t *net = network_get_network();
            for (int i = 0; i < net->count; i++)
                net->peers[i].uncertainty = (float)(rand() % 1000) / 1000.0f;
  

    }

    /* Pad to exactly 10ms from entry regardless of which path was taken */
    uint32_t elapsed = osKernelGetTickCount() - t_start;
    if (elapsed < 10U) osDelay(10U - elapsed);
}

uint64_t position_calibrate_timestamp(uint64_t orig_timestamp){
    return orig_timestamp-1;
}