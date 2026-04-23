#pragma once

#include "../UWB_app/uwb_exchange.h"   /* uwb_etwr_result_t */
#include "../UWB_app/messages.h"

void distance_calculate(uwb_etwr_result_t result);

uint64_t position_calibrate_timestamp(uint64_t orig_timestamp);

//#define DEBUG_distance_populate

#define UWB_40BIT_MASK 0xFFFFFFFFFFULL

#define SPEED_OF_LIGHT   299702547.0
#define METERS_PER_TICK  ((float)(DWT_TIME_UNITS * SPEED_OF_LIGHT)) /* ~4.6917e-3 */

#define MAX_PASSIVE        (NETWORK_MAX_PEERS - 2)          /* 5 */
#define MAX_SECOND_ORDER   (2 * MAX_PASSIVE)                /* 10 */
#define MAX_THIRD_ORDER    (MAX_PASSIVE * (MAX_PASSIVE - 1) / 2)  /* 10 */

#define DIST_SHARE_MAX_M          200.0
#define DIST_SHARE_DIST_MAX_TICKS (DIST_SHARE_MAX_M / (SPEED_OF_LIGHT * DWT_TIME_UNITS))
#define DIST_SHARE_TICKS_PER_LSB  (DIST_SHARE_DIST_MAX_TICKS / 65535.0)
#define DWT_TICK_TO_US            (1.0f / 63897.6f)


/* vel_vert  : signed ±4 m/s, offset-binary (direction matters)       */
#define VEL_VERT_RANGE_MS     4.0f
#define VEL_VERT_MS_PER_LSB   (2.0f * VEL_VERT_RANGE_MS / 255.0f)   /* ~31.4 mm/s */

/* vel_horiz : unsigned 0..4 m/s (direction irrelevant, 2x resolution) */
#define VEL_HORIZ_MAX_MS      4.0f
#define VEL_HORIZ_MS_PER_LSB  (VEL_HORIZ_MAX_MS / 255.0f)            /* ~15.7 mm/s */


/*
 * CERTAINTY SYSTEM — UWB Position Measurement Quality Score
 *
 * Produces a uint8_t score [0..254] each measurement round (200 ms).
 * Score is recomputed fresh every round — no accumulation.
 *
 * Score anchors:
 *   0   — node not heard this round
 *   127 — SS-TWR, average turnaround, average LOS  (the "neutral" node)
 *   254 — DS-TWR, perfect LOS
 */

/* ------------------------------------------------------------------ */
/* TUNING DEFINES — adjust these to calibrate your environment         */
/* ------------------------------------------------------------------ */

/*
 * Base certainty scores per measurement method.
 * These are the starting values BEFORE timing and LOS adjustments.
 * Gap between methods must exceed the max combined adjustment range
 * so that a better method always outranks a worse one.
 *
 *   DS-TWR  base: 224   (+30 LOS headroom  → 254 max)
 *   SS-TWR  base: 127   (neutral midpoint  → 127 at average conditions)
 *   TDOA    base:  60   (indirect geometry, lower ceiling)
 */
#define CERTAINTY_BASE_DSTWR     224
#define CERTAINTY_BASE_SSTWR     127
#define CERTAINTY_BASE_TDOA       60

/*
 * SS-TWR timing adjustment — linear, centered at CERTAINTY_TREF_US.
 *
 * CERTAINTY_TREF_US: turnaround time (µs) considered "average".
 *   At this value timing offset = 0.
 *   Faster → positive bonus (up to CERTAINTY_TIMING_MAX).
 *   Slower → negative penalty (down to CERTAINTY_TIMING_MIN).
 *
 * CERTAINTY_KT: slope in offset-units per µs deviation.
 *   Example: KT = 0.01 means 1000 µs off → ±10 points.
 */
#define CERTAINTY_TREF_US        1000.0f
#define CERTAINTY_KT             0.015f
#define CERTAINTY_TIMING_MAX      15
#define CERTAINTY_TIMING_MIN     -30

/*
 * LOS adjustment — sqrt nonlinearity, centered at CERTAINTY_DELTA0_DB.
 *
 * CERTAINTY_DELTA0_DB: the RSSI-FP gap (dB) considered "average" LOS.
 *   At this value LOS offset = 0.
 *   Below → positive bonus (good LOS), up to CERTAINTY_LOS_MAX.
 *   Above → penalty (NLOS), steep in first ~10 dB then flattening,
 *            down to CERTAINTY_LOS_MIN.
 *
 * CERTAINTY_KN: controls steepness of the sqrt curve.
 *   KN = 9.5 gives ~-30 penalty at 10 dB above average.
 */
#define CERTAINTY_DELTA0_Q8      1024
#define CERTAINTY_KN_Q8          0.594f
#define CERTAINTY_LOS_MAX         30
#define CERTAINTY_LOS_MIN        -40

/* ------------------------------------------------------------------ */
/* TYPES                                                               */
/* ------------------------------------------------------------------ */

typedef enum {
    MEAS_NONE       = 0,   /* No measurement received this round      */
    MEAS_TDOA_SSTWR = 1,   /* TDOA aided by SS-TWR                    */
    MEAS_SSTWR      = 2,   /* Single-sided two-way ranging            */
    MEAS_DSTWR      = 3,   /* Double-sided two-way ranging (best)     */
} MeasurementType;

/*
 * @brief Timestamps captured during an active DS-TWR exchange.
 *
 * Role determines which fields are populated:
 *
 * | Role       | Valid fields                  |
 * |------------|-------------------------------|
 * | Initiator  | poll_tx, resp_rx, final_tx    |
 * | Responder  | poll_rx, resp_tx, final_rx    |
 *
 * Unused fields are zero-initialised and must not be read.
 */
typedef struct {
    uint64_t      poll_tx;  /**< Initiator: time POLL was transmitted. */
    uwb_rx_meas_t poll_rx;  /**< Responder: POLL reception measurement. */

    uint64_t      resp_tx;  /**< Responder: time RESPONSE was transmitted. */
    uwb_rx_meas_t resp_rx;  /**< Initiator: RESPONSE reception measurement. */

    uint64_t      final_tx; /**< Initiator: scheduled FINAL TX time (predicted). */
    uwb_rx_meas_t final_rx; /**< Responder: FINAL reception measurement. */
} twr_timestamps_t;


typedef struct {
    uwb_rx_meas_t poll_rx;  /**< Reception measurement of the POLL frame. */
    uwb_rx_meas_t resp_rx;  /**< Reception measurement of the RESPONSE frame. */
} twr_observation_t;


typedef struct {
    uint64_t      init_tx; 
    uwb_rx_meas_t init_rx; 

    uint64_t      answer_tx; 
    uwb_rx_meas_t answer_rx; 
} ss_twr_t;

typedef struct {
    uint64_t      init_tx; 
    uwb_rx_meas_t init_rx; 

    uwb_rx_meas_t master_init_rx; 

    uint64_t      answer_tx; 
    uwb_rx_meas_t answer_rx; 
} tdoa_twr_t;

typedef struct {
    uint16_t initiator_id;
    uint16_t responder_id;

    twr_timestamps_t twr;              
    double result_distance_tick;  /* populated after calculation */
} first_order_t;

typedef struct {
    uint16_t initiator_id;
    uint16_t responder_id;

    twr_observation_t twr_observation; 

    ss_twr_t twr;     
    
    double result_distance_tick;

} second_order_t;

typedef struct {
    uint16_t initiator_id;
    uint16_t responder_id;

    twr_observation_t twr_observation_c;
    twr_observation_t twr_observation_d; 

    ss_twr_t twr; 

    double result_distance_tick;

}  third_order_t;

typedef struct {

    first_order_t  first;                              // always exactly 1

    second_order_t second[MAX_SECOND_ORDER];
    uint8_t        second_count;

    third_order_t  third[MAX_THIRD_ORDER]; 
    uint8_t        third_count;

} timestamps_t;


uint16_t dist_ticks_to_scale(double ticks);
double dist_scale_to_ticks(uint16_t encoded);
uint8_t vel_vert_to_u8(float v);
float vel_vert_u8_to_ms(uint8_t u);
uint8_t vel_horiz_to_u8(float v);
float vel_horiz_u8_to_ms(uint8_t u);