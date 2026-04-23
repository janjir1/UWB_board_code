#pragma once

#include <stdint.h>

#define CFO_RAW_TO_PPM(raw)  ((float)(raw) / (float)(1 << 26) * 1e6f)
#define PPM_TO_K(ppm)        (1.0f + (ppm) / 1e6f)
#define CFO_RAW_TO_K(raw)    PPM_TO_K(CFO_RAW_TO_PPM(raw))

/* RX timestamp power correction coefficient.
 * Model: delta_ticks = CALIB_RX_TS_A * rssi_dbm
 *
 * Corrects leading-edge detection bias that varies with signal level.
 * Assumes static antenna delay is already calibrated (dwt_setrxantennadelay).
 * Typical range: -0.1 to -0.5 ticks per dBm. Must be measured per board. */
#define CALIB_RX_TS_A   0.0f   /* TODO: measure — ticks per dBm */

/* 3 metres expressed in DW3000 ticks (one-way, two-way already halved in DS-TWR):
 * ticks = metres / (c / (499.2e6 * 128 * 2)) */
#define CALIB_ANTENNA_MIN_DIST_TICKS \
    (3.0 * (499.2e6 * 128.0 * 2.0) / 299792458.0)

/* Device tilt (from vertical) above which correction is skipped entirely.
 * 15° keeps the board nearly flat — null axis is close to true vertical,
 * donut approximation is most valid here. */
#define CALIB_ANTENNA_TILT_LIMIT_DEG        15.0f

/* Within TILT_LIMIT, if the target elevation is within this many degrees
 * of the null axis elevation — flag unreliable instead of correcting.
 * Tight window: only trigger when alignment is very close to the null. */
#define CALIB_ANTENNA_SIMILARITY_DEG        10.0f

/* Target elevation above which the donut null correction is applied.
 * Below this angle the antenna is near its gain maximum — no correction.
 * 70° is conservative — only correct deep in the null zone. */
#define CALIB_ANTENNA_NULL_THRESHOLD_DEG    70.0f

/* Donut pattern exponent. n=2 for dipole-like pattern. */
#define CALIB_ANTENNA_N                     2.0f

/* Timestamp correction slope for antenna null region (ticks per dB).
 * TODO: measure empirically. Zero = math runs but no correction applied. */
#define CALIB_ANTENNA_A                     0.0f

/**
 * @brief Overwrite k from scratch using a single CFO sample.
 * @param cfo  Raw dwt_readclockoffset() value.
 */
void calibrate_set_clock_offset_sync(int16_t cfo);

/**
 * @brief Refine k by averaging it with a new CFO sample.
 *        On the first call (uninitialised), behaves like sync.
 * @param cfo  Raw dwt_readclockoffset() value.
 */
void calibrate_set_clock_offset_poll(int16_t cfo);

/**
 * @brief Return the current clock correction factor k.
 *        Returns 1.0f if no calibration has been performed yet.
 */
float calibrate_get_k(void);

void calibrate_set_pitch(float pitch_rad);

uint64_t calibrate_rx_timestamp(uint64_t rx_timestamp,
                                int16_t rssi_q8,
                                uint16_t target_id,
                                bool *antenna_unreliable);

uint64_t calibrate_tx_timestamp(uint64_t tx_timestamp);

