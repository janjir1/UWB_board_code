#include "cmsis_os.h"
#include "cmsis_os2.h"
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <float.h>
#include <stdlib.h>

#include "../Generic/my_print.h"
#include "calibrate.h"
#include "../UWB_app/uwb_network.h"
#include "distance.h"

static float s_k         = 0.0f;
static float s_pitch_rad = 0.0f;

/**
 * @brief Update the clock-rate correction factor from a SYNC CFO reading.
 *
 * Replaces the stored @c s_k directly. Use this on the first CFO sample
 * of a round, before any POLL-based averaging has occurred.
 *
 * @param cfo  Raw carrier-frequency-offset value from the DWM3000 RX frame.
 */
void calibrate_set_clock_offset_sync(int16_t cfo)
{
    s_k = CFO_RAW_TO_K(cfo);
    mprintf("CFO sync: k = %.9f\r\n", s_k);
}

/**
 * @brief Refine the clock-rate correction factor from a POLL CFO reading.
 *
 * On the first call after boot (@c s_k == 0) the value is set directly.
 * On subsequent calls it is averaged with the current estimate. The @c 0.0f
 * sentinel is safe because @c k is a clock-rate ratio ~= 1 ± 20 ppm and
 * can never legitimately equal zero.
 *
 * @param cfo  Raw carrier-frequency-offset value from the DWM3000 RX frame.
 */
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

/**
 * @brief Return the current clock-rate correction factor.
 *
 * Returns 1.0 when no CFO sample has been received yet, so timestamps
 * pass through uncorrected rather than being scaled to zero.
 *
 * @return Stored @c k value, or 1.0f if uninitialised.
 */
float calibrate_get_k(void)
{
    return (s_k == 0.0f) ? 1.0f : s_k;
}

/**
 * @brief Store the IMU pitch angle used for antenna pattern correction.
 *
 * A value of 0.0f disables antenna correction in @ref calibrate_rssi_antenna
 * and @ref calibrate_rx_timestamp.
 *
 * @param pitch_rad  Pitch angle in radians from the IMU.
 */
void calibrate_set_pitch(float pitch_rad)
{
    s_pitch_rad = pitch_rad;
}

/**
 * @brief Apply antenna pattern gain correction to a raw RSSI value.
 *
 * Models the dipole-like @c cos^N(@f$\theta@f$) gain pattern of the UWB antenna.
 * The elevation angle to the target is computed from stored EKF positions.
 * If the target is near the antenna null (elevation close to
 * @c 90° - pitch), the measurement is flagged as unreliable and the
 * uncorrected RSSI is returned.
 *
 * Returns the uncorrected @p rssi_dbm unchanged when:
 * - IMU pitch has not been initialised (@c s_pitch_rad == 0),
 * - the target peer is not in the active peer list,
 * - the stored distance is invalid or below @c CALIB_ANTENNA_MIN_DIST_TICKS,
 * - the inter-node separation is below 1 mm.
 *
 * @param[in]  rssi_dbm           Raw RSSI converted to dBm.
 * @param[in]  target_id          Node ID of the transmitting peer.
 * @param[out] antenna_unreliable Set to true if the target is near the antenna null.
 * @return Corrected RSSI in dBm, or @p rssi_dbm if correction is skipped.
 */
float calibrate_rssi_antenna(float    rssi_dbm,
                              uint16_t target_id,
                              bool    *antenna_unreliable)
{
    *antenna_unreliable = false;

    /* IMU pitch not yet initialised — skip antenna correction entirely. */
    if (s_pitch_rad == 0.0f) return rssi_dbm;

    const network_t *net = network_get_network();

    int8_t idx = network_get_peer_index(target_id);
    if (idx < 0) return rssi_dbm;

    double dist_ticks = dist_scale_to_ticks(
        network_get_distance(net->self.id, target_id));

    if (dist_ticks < 0.0)                          return rssi_dbm;
    if (dist_ticks < CALIB_ANTENNA_MIN_DIST_TICKS) return rssi_dbm;

    const float *self_pos   = net->self.pos;
    const float *target_pos = net->peers[idx].pos;

    float dx = target_pos[0] - self_pos[0];
    float dy = target_pos[1] - self_pos[1];
    float dz = target_pos[2] - self_pos[2];
    float pos_dist = sqrtf(dx*dx + dy*dy + dz*dz);

    if (pos_dist < 1e-3f) return rssi_dbm;

    float elevation_deg  = fabsf(asinf(dz / pos_dist) * (180.0f / (float)M_PI));
    float pitch_deg      = s_pitch_rad * (180.0f / (float)M_PI);
    float null_elev_deg  = 90.0f - pitch_deg;

    if (pitch_deg > CALIB_ANTENNA_TILT_LIMIT_DEG &&
        fabsf(null_elev_deg - elevation_deg) < CALIB_ANTENNA_SIMILARITY_DEG) {
        *antenna_unreliable = true;
        return rssi_dbm;
    }

    if (elevation_deg < CALIB_ANTENNA_NULL_THRESHOLD_DEG)
        return rssi_dbm;

    float theta_rad  = elevation_deg * ((float)M_PI / 180.0f);
    float cos_theta  = cosf(theta_rad);
    if (cos_theta < 1e-6f) cos_theta = 1e-6f;

    float gain_loss_db = CALIB_ANTENNA_N * 10.0f * log10f(cos_theta);

    return rssi_dbm - gain_loss_db;
}

/**
 * @brief Apply full calibration pipeline to a raw RX timestamp.
 *
 * Performs three sequential corrections:
 * -# Antenna pattern correction via @ref calibrate_rssi_antenna (skipped
 *    if IMU pitch is uninitialised).
 * -# Power-dependent leading-edge correction: @c @f$\delta@f$ = CALIB_RX_TS_A × RSSI_dBm.
 * -# Clock-rate correction using the CFO-derived factor @c k.
 *
 * @note The clock-rate multiplication uses @c double to preserve all 40 bits
 *       of the DWM3000 timestamp. A @c float cast would discard the lower
 *       ~16 bits (~150 m equivalent), and this noise does not cancel when
 *       two calibrated timestamps are subtracted in @c distance.c.
 *
 * @param[in]  rx_timestamp       Raw 40-bit RX timestamp from the DWM3000.
 * @param[in]  rssi_q8            Received signal strength in Q8 fixed-point (dBm × 256).
 * @param[in]  target_id          Node ID of the transmitting peer.
 * @param[out] antenna_unreliable Set to true if the antenna null flag was triggered.
 * @return Fully calibrated RX timestamp.
 */
uint64_t calibrate_rx_timestamp(uint64_t  rx_timestamp,
                                 int16_t   rssi_q8,
                                 uint16_t  target_id,
                                 bool     *antenna_unreliable)
{
    float rssi_dbm = (float)rssi_q8 / 256.0f;

    /* Step 1: antenna pattern correction — adjust RSSI to isotropic equivalent.
     * Skipped automatically if s_pitch_rad == 0.0f (IMU not yet set). */
    rssi_dbm = calibrate_rssi_antenna(rssi_dbm, target_id, antenna_unreliable);

    /* Step 2: power-dependent leading-edge correction using corrected RSSI. */
    float    delta = CALIB_RX_TS_A * rssi_dbm;
    uint64_t ts    = rx_timestamp + (int64_t)delta;

    /* Step 3: clock rate correction using CFO-derived k.
     * Use double — see @ref calibrate_rx_timestamp note for rationale. */
    ts = (uint64_t)((double)ts * (double)calibrate_get_k());

    return ts;
}

/**
 * @brief Apply clock-rate correction to a raw TX timestamp.
 *
 * Scales the timestamp by the CFO-derived factor @c k using @c double
 * arithmetic for the same precision reasons as @ref calibrate_rx_timestamp.
 *
 * @param tx_timestamp  Raw 40-bit TX timestamp from the DWM3000.
 * @return Clock-rate corrected TX timestamp.
 */
uint64_t calibrate_tx_timestamp(uint64_t tx_timestamp)
{
    return (uint64_t)((double)tx_timestamp * (double)calibrate_get_k());
}