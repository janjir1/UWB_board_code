#include "cmsis_os.h"
#include "cmsis_os2.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

#include "../Generic/my_print.h"
#include "calibrate.h"
#include "../UWB_app/uwb_network.h"
#include "distance.h"

static float s_k = 0.0f;
static float s_pitch_rad = 0.0f;

//TODO we are not yet calibrating POLL and RESPONSE frames, only FINAL and PASSIVE

void calibrate_set_clock_offset_sync(int16_t cfo)
{
    s_k = CFO_RAW_TO_K(cfo);
    mprintf("CFO sync: k = %.9f\r\n", s_k);
}

void calibrate_set_clock_offset_poll(int16_t cfo)
{
    float k_new = CFO_RAW_TO_K(cfo);

    if (s_k == 0.0f) {
        s_k = k_new;
    } else {
        s_k = (s_k + k_new) * 0.5f;
    }

     mprintf("CFO poll: k = %.9f\r\n", s_k);
}

float calibrate_get_k(void)
{
    return (s_k == 0.0f) ? 1.0f : s_k;
}

void calibrate_set_pitch(float pitch_rad)
{
    s_pitch_rad = pitch_rad;
}

float calibrate_rssi_antenna(float rssi_dbm,
                             uint16_t target_id,
                             bool *antenna_unreliable)
{
    *antenna_unreliable = false;

    /* IMU pitch not yet initialised — skip antenna correction entirely */
    if (s_pitch_rad == 0.0f) return rssi_dbm;

    const network_t *net = network_get_network();

    int8_t idx = network_get_peer_index(target_id);
    if (idx < 0) return rssi_dbm;

    double dist_ticks = dist_scale_to_ticks(
        network_get_distance(net->self.id, target_id));

    if (dist_ticks < 0.0) return rssi_dbm;
    if (dist_ticks < CALIB_ANTENNA_MIN_DIST_TICKS) return rssi_dbm;

    const float *self_pos   = net->self.pos;
    const float *target_pos = net->peers[idx].pos;

    float dx = target_pos[0] - self_pos[0];
    float dy = target_pos[1] - self_pos[1];
    float dz = target_pos[2] - self_pos[2];
    float pos_dist = sqrtf(dx*dx + dy*dy + dz*dz);

    if (pos_dist < 1e-3f) return rssi_dbm;

    float elevation_deg = fabsf(asinf(dz / pos_dist) * (180.0f / (float)M_PI));
    float pitch_deg     = s_pitch_rad * (180.0f / (float)M_PI);
    float null_elev_deg = 90.0f - pitch_deg;

    if (pitch_deg > CALIB_ANTENNA_TILT_LIMIT_DEG &&
        fabsf(null_elev_deg - elevation_deg) < CALIB_ANTENNA_SIMILARITY_DEG) {
        *antenna_unreliable = true;
        return rssi_dbm;
    }

    if (elevation_deg < CALIB_ANTENNA_NULL_THRESHOLD_DEG)
        return rssi_dbm;

    float theta_rad    = elevation_deg * ((float)M_PI / 180.0f);
    float cos_theta    = cosf(theta_rad);
    if (cos_theta < 1e-6f) cos_theta = 1e-6f;

    float gain_loss_db = CALIB_ANTENNA_N * 10.0f * log10f(cos_theta);

    return rssi_dbm - gain_loss_db;
}

uint64_t calibrate_rx_timestamp(uint64_t rx_timestamp,
                                int16_t rssi_q8,
                                uint16_t target_id,
                                bool *antenna_unreliable)
{

    float rssi_dbm = (float)rssi_q8 / 256.0f;

    /* Step 1: antenna pattern correction — adjust RSSI to isotropic equivalent.
     * Skipped automatically inside if s_pitch_rad == 0.0f (IMU not yet set). */
    rssi_dbm = calibrate_rssi_antenna(rssi_dbm, target_id, antenna_unreliable);

    /* Step 2: power-dependent leading-edge correction using corrected RSSI */
    float delta = CALIB_RX_TS_A * rssi_dbm;
    uint64_t ts = rx_timestamp + (int64_t)delta;

    /* Step 3: clock rate correction using CFO-derived k */
    ts = (uint64_t)((float)ts * calibrate_get_k());

    return ts;
}

uint64_t calibrate_tx_timestamp(uint64_t tx_timestamp)
{
    return (uint64_t)((float)tx_timestamp * calibrate_get_k());
}
