/**
 * @file uwb_exchange.h
 * @brief Public API and configuration constants for the UWB ranging exchange.
 *
 * Provides the synchronisation (@ref uwb_sync), extended two-way ranging
 * (@ref uwb_extended_twr), and result-sharing (@ref uwb_share) interfaces
 * used by the top-level network task.
 *
 * @par Timing units
 * - Constants ending in `_MS` or `ms` are RTOS ticks (1 ms).
 * - Constants ending in `_TICKS` are DW3xxx 40-bit counter units (~15.65 ps).
 */

#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* -----------------------------------------------------------------------
 * Timing constants
 * ----------------------------------------------------------------------- */

/**
 * @defgroup uwb_timing Timing constants
 * @{
 */

/** @brief DW3xxx deep-sleep duration (ms). */
#define DEEP_SLEEP  200U

/** @brief Slave RX window for the initial SYNC broadcast (ms). */
#define T_SYNC_RX_SET       200U

/** @brief Master RX window waiting for slave join replies after SYNC (ms). */
#define T_SYNC_RX_ANSWER    10U

/** @brief Slave RX window waiting for a SHARE broadcast (ms). */
#define T_SHARE_RX_WAIT     50U

/** @brief Early-wakeup margin before a scheduled radio event (ms). */
#define T_EARLY_WKUP        5U

/** @brief Convert microseconds to DW3xxx 40-bit timestamp ticks. */
#define UWB_US_TO_TICKS(us) ((uint64_t)((us) * 63898ULL))

/** @brief Master RX window waiting for RESPONSE after POLL (ms). */
#define T_RESPONSE_RX_WAIT  100U

/** @brief Slave RX window waiting for FINAL (ms). */
#define T_FINAL_RX_WAIT     100U

/** @brief Slave RX window covering the entire TWR exchange (ms). */
#define T_TWR_RX_WAIT       300U

/** @brief Delay from RESPONSE reception to FINAL transmission (ticks). */
#define T_FINAL_TX_ASAP_TICKS  UWB_US_TO_TICKS(800)

/** @brief Minimum delay before a passive observer's TX (ticks). */
#define T_PASSIVE_TX_ASAP_TICKS UWB_US_TO_TICKS(1000)

/** @brief Timeout waiting for TXFRS after scheduling the delayed FINAL (ms). */
#define T_FINAL_TX_WAIT_MS      10U

/** @brief Timeout waiting for TXFRS after scheduling the passive TX (ms). */
#define T_PASSIVE_TX_WAIT_MS    10U

/** @brief Consecutive SYNC timeouts before a slave promotes itself to master. */
#define SYNC_TIMEOUT_MAX        5U

/** @brief Broadcast destination address. */
#define ALL_ID  0xFFFFU

/** @} */ /* end of uwb_timing */

/* -----------------------------------------------------------------------
 * Result enumerations
 * ----------------------------------------------------------------------- */

/**
 * @brief Result codes returned by uwb_sync().
 */
typedef enum {
    UWB_SYNC_MASTER,            /**< Master: SYNC sent, at least one peer acknowledged. */
    UWB_SYNC_MASTER_NO_REPLY,   /**< Master: SYNC sent, no replies received in window. */
    UWB_SYNC_SLAVE_ACKNOWLEDGED,/**< Slave: own ID confirmed in master's peer list. */
    UWB_SYNC_SLAVE_PENDING,     /**< Slave: join reply sent, awaiting next SYNC. */
    UWB_SYNC_SLAVE_REPLY_FAILED,/**< Slave: join reply TX failed on both attempts. */
    UWB_SYNC_NEW_MASTER,        /**< Slave promoted itself to master (no master heard). */
    UWB_SYNC_NEW_SLAVE,         /**< Master yielded role to a higher-ID master. */
    UWB_SYNC_TX_FAILED,         /**< Master: SYNC TX failed at the hardware level. */
} uwb_sync_result_t;

/**
 * @brief Result codes returned by uwb_extended_twr().
 */
typedef enum {
    /* --- Success --- */
    UWB_TWR_EXCHANGE_COMPLETE,  /**< Master: POLL→FINAL sent; master role passed to target. */
    UWB_TWR_RECEIVED,           /**< Responder: FINAL received; timestamps ready for ranging. */
    UWB_TWR_RECEIVED_PASSIVE,   /**< Passive observer: full exchange captured. */
    UWB_TWR_NOTHING,            /**< Exchange completed but this device was not involved. */
    /* --- Failure --- */
    UWB_TWR_NOT_ENOUGH_DEVICES, /**< No peers available or sync state invalid. */
    UWB_TWR_TX_FAILED,          /**< TX failed. */
    UWB_TWR_TIMEOUT,            /**< RX window expired before the expected message arrived. */
    UWB_TWR_UNEXPECTED_MASTER,  /**< Frame received from an unexpected sender. */
} uwb_etwr_result_t;

/* -----------------------------------------------------------------------
 * Public API
 * ----------------------------------------------------------------------- */

/**
 * @brief Perform one synchronisation round.
 * @return Outcome and role for the subsequent TWR phase.
 */
uwb_sync_result_t uwb_sync(void);

/**
 * @brief Execute one extended two-way ranging exchange.
 * @param sync_result Result from the preceding @ref uwb_sync call.
 * @return Success or failure mode.
 */
uwb_etwr_result_t uwb_extended_twr(uwb_sync_result_t sync_result);

/**
 * @brief Broadcast or receive the ranging result and compute sleep time.
 * @param etwr_result Result from the preceding @ref uwb_extended_twr call.
 * @param sleep_time  Nominal sleep duration for this cycle (ms).
 * @return Actual sleep duration (ms) adjusted for timing drift.
 */
uint32_t uwb_share(uwb_etwr_result_t etwr_result, uint32_t sleep_time);

#ifdef __cplusplus
}
#endif