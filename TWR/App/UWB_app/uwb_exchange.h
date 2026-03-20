#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* -----------------------------------------------------------------------
 * Timing constants
 * All ms values are RTOS ticks — assumes 1 tick = 1 ms (1000 Hz).
 * All _TICKS values are DW3xxx 40-bit counter units (~15.65 ps per tick).
 * TODO: Tune all constants against real hardware measurements.
 * ----------------------------------------------------------------------- */
#define T_PERIOD = 500
#define DEEP_SLEEP = 500
/** @brief Slave RX window for the initial SYNC broadcast.
 *  Must be longer than the longest inter-sync sleep interval. */
#define T_SYNC_RX_SET          1000U          /* ms */

//TODO: Tune all constants against real hardware measurements. - dont do that, i gave up

/** @brief Master RX window waiting for slave join replies after SYNC.
 *  Shorter = faster sync cycle. Must cover worst-case slave reply delay
 *  (id-based stagger = up to 9 ms) plus one UWB frame air time (~0.3 ms).
 *  TODO: reduce once per-device stagger is tuned. */
#define T_SYNC_RX_ANSWER       20U            /* ms */

#define T_SHARE_RX_WAIT         50U

#define T_EARLY_WKUP            5U

/** @brief Slave delay after detecting a POLL — avoids corrupting an
 *  ongoing TWR exchange before transmitting own SYNC reply.
 *  TODO: set to actual POLL→FINAL duration once measured. */
#define T_POLL_TO_SYNC         510U            /* ms — placeholder */

/** @brief Slave delay after detecting a RESPONSE.
 *  TODO: set to actual RESPONSE→FINAL duration once measured. */
#define T_RESPONSE_TO_SYNC     508U             /* ms — placeholder */

/** @brief Slave delay after detecting a FINAL.
 *  TODO: set to FINAL→next-SYNC guard time once measured. */
#define T_FINAL_TO_SYNC        505U             /* ms — placeholder */

/** @brief Slave delay after detecting another slave's SYNC reply.
 *  Prevents the next join attempt from colliding with an ongoing reply. */
#define T_SYNC_TO_SYNC         502U             /* ms — placeholder */

/** @brief Convert UWB microseconds to DW3xxx 40-bit timestamp ticks.
 *  1 tick = 1 / (499.2 MHz × 128) ≈ 15.65 ps  →  63 898 ticks/µs. */
#define UWB_US_TO_TICKS(us)    ((uint64_t)((us) * 63898ULL))

/** @brief Master RX window waiting for RESPONSE after POLL.
 *  Must exceed worst-case slave processing + TX scheduling time (~2 ms).
 *  TODO: reduce to ~20 ms once timing is stable. */
#define T_RESPONSE_RX_WAIT     100U           /* ms — currently conservative for testing */

/** @brief Slave RX window covering the entire TWR exchange.
 *  Must exceed POLL air time + FINAL delay + FINAL air time + passive chain.
 *  TODO: reduce to ~60 ms once passive chain length is known. */
#define T_TWR_RX_WAIT          300U           /* ms — currently conservative for testing */

/** @brief Delay from RESPONSE reception to FINAL transmission (ticks).
 *  Must give the master CPU enough time to decode, encode, and schedule
 *  the delayed TX. 3 000 µs is safe for Cortex-M4 at 64–168 MHz.
 *  TODO: reduce to 1 500 µs if CPU budget allows, to lower ranging latency. */
#define T_FINAL_TX_DELAY_TICKS UWB_US_TO_TICKS(3000)   /* ≈ 191 694 000 ticks */

/** @brief Timeout waiting for TXFRS event after scheduling the delayed FINAL.
 *  Must exceed T_FINAL_TX_DELAY in ms plus OS scheduling jitter. */
#define T_FINAL_TX_WAIT_MS     10U            /* ms */

/** @brief Number of consecutive SYNC timeouts before a slave self-promotes
 *  to master. Keep small (2–3) to recover quickly from a missing master.
 *  TODO: increase to 3–5 in larger networks to reduce false promotions. */
#define SYNC_TIMEOUT_MAX       2U

/** @brief Broadcast address — all devices accept frames addressed here. */
#define ALL_ID                 0xFFFFU

/* -----------------------------------------------------------------------
 * Result enumerations
 * ----------------------------------------------------------------------- */

/**
 * @brief Result codes returned by uwb_sync().
 */
typedef enum {
    UWB_SYNC_MASTER,              /**< Master: SYNC sent, one peer acknowledged. */
    UWB_SYNC_MASTER_NO_REPLY,     /**< Master: SYNC sent, no replies in window. */
    UWB_SYNC_SLAVE_ACKNOWLEDGED,  /**< Slave: own ID confirmed in master's peer list. */
    UWB_SYNC_SLAVE_PENDING,       /**< Slave: join reply sent, awaiting next SYNC. */
    UWB_SYNC_SLAVE_REPLY_FAILED,  /**< Slave: join reply TX failed twice. */
    UWB_SYNC_NEW_MASTER,          /**< Slave promoted itself to master (no master heard). */
    UWB_SYNC_NEW_SLAVE,           /**< Master yielded to a higher-ID master. */
    UWB_SYNC_TX_FAILED,           /**< Master SYNC TX failed at hardware level. */
} uwb_sync_result_t;

/**
 * @brief Result codes returned by uwb_extended_twr().
 */
typedef enum {
    /* --- Success --- */
    UWB_TWR_EXCHANGE_COMPLETE,    /**< Master: POLL→FINAL sent, master role passed to target. */
    UWB_TWR_RECEIVED,             /**< Responder: FINAL received, timestamps ready for ranging. */
    UWB_TWR_RECEIVED_PASSIVE,     /**< Passive observer: full exchange captured, ready for TDoA. */
    UWB_TWR_NOTHING,              /**< Exchange completed but this device was not involved. */
    /* --- Failure --- */
    UWB_TWR_NOT_ENOUGH_DEVICES,   /**< No peers available or sync state invalid. */
    UWB_TWR_TX_FAILED,            /**< TX of POLL, RESPONSE, or FINAL failed. */
    UWB_TWR_TIMEOUT,              /**< RX window expired before expected message arrived. */
    UWB_TWR_UNEXPECTED_MASTER,    /**< Message received from an unexpected sender. */
} uwb_etwr_result_t;

/* -----------------------------------------------------------------------
 * Public API
 * ----------------------------------------------------------------------- */

uwb_sync_result_t  uwb_sync(uint8_t seq_num);
uwb_etwr_result_t  uwb_extended_twr(uint8_t seq_num, uwb_sync_result_t sync_result);
uwb_etwr_result_t uwb_twr_test(uint8_t seq_num, uwb_sync_result_t sync_result);
uint32_t uwb_share (uint8_t seq_num, uwb_etwr_result_t etwr_result, uint32_t sleep_time);

#ifdef __cplusplus
}
#endif
