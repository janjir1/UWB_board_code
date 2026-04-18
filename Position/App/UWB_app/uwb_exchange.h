/**
 * @file uwb_exchange.h
 * @brief Public API and configuration constants for the UWB ranging exchange.
 *
 * Provides the synchronisation (@ref uwb_sync), extended two-way ranging
 * (@ref uwb_extended_twr), and result-sharing (@ref uwb_share) interfaces
 * used by the top-level network task.
 *
 * @par Timing units
 * - Constants whose names end with `_MS` or have a `ms` comment are RTOS ticks
 *   (1 tick = 1 ms, 1 000 Hz SysTick).
 * - Constants whose names end with `_TICKS` are DW3xxx 40-bit counter units
 *   (~15.65 ps per tick, i.e. ~63 898 ticks/µs).
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


/**
 * @brief DW3xxx deep-sleep duration (ms).
 *
 * How long the radio is held in deep-sleep inside each cycle. Must be
 * less than T_PERIOD to leave time for the active exchange phase.
 */
#define DEEP_SLEEP  200U    /* ms */

/**
 * @brief Slave RX window for the initial SYNC broadcast (ms).
 *
 * Must be longer than the longest inter-sync sleep interval so a slave
 * that wakes up at an arbitrary time still catches the SYNC frame.
 */
#define T_SYNC_RX_SET       1000U   /* ms */

/**
 * @brief Master RX window waiting for slave join replies after SYNC (ms).
 *
 * Shorter values speed up the sync cycle but must still cover the
 * worst-case slave reply delay (ID-based stagger ≤ 9 ms) plus one UWB
 * frame air time (~0.3 ms).
 *
 * @todo Reduce once per-device stagger is tuned.
 */
#define T_SYNC_RX_ANSWER    10U     /* ms */

/**
 * @brief Slave RX window waiting for a SHARE broadcast (ms).
 *
 * Must cover the time from the end of the TWR exchange until the master
 * transmits the SHARE frame.
 */
#define T_SHARE_RX_WAIT     50U     /* ms */

/**
 * @brief Early-wakeup margin before a scheduled radio event (ms).
 *
 * The MCU wakes this many ms before the next expected TX/RX to allow
 * HAL and driver initialisation to complete in time.
 */
#define T_EARLY_WKUP        5U      /* ms */

/**
 * @brief Slave back-off after receiving a POLL (ms).
 *
 * Prevents the slave's SYNC reply from colliding with an ongoing
 * POLL → FINAL exchange. Set conservatively until actual
 * POLL → FINAL duration is measured.
 *
 * @todo Replace with measured POLL→FINAL duration.
 */
#define T_POLL_TO_SYNC      510U    /* ms — placeholder */

/**
 * @brief Slave back-off after receiving a RESPONSE (ms).
 *
 * Guards against transmitting a SYNC reply while a RESPONSE → FINAL
 * exchange is still in progress.
 *
 * @todo Replace with measured RESPONSE→FINAL duration.
 */
#define T_RESPONSE_TO_SYNC  508U    /* ms — placeholder */

/**
 * @brief Slave back-off after receiving a FINAL (ms).
 *
 * Provides a guard time between the end of a TWR exchange and the
 * slave's next SYNC reply attempt.
 *
 * @todo Replace with measured FINAL→next-SYNC guard time.
 */
#define T_FINAL_TO_SYNC     505U    /* ms — placeholder */

/**
 * @brief Slave back-off after receiving another slave's SYNC reply (ms).
 *
 * Prevents back-to-back SYNC replies from colliding on the channel.
 */
#define T_SYNC_TO_SYNC      502U    /* ms — placeholder */

/**
 * @brief Convert microseconds to DW3xxx 40-bit timestamp ticks.
 *
 * Conversion factor: 1 tick = 1 / (499.2 MHz × 128) ≈ 15.65 ps,
 * giving approximately 63 898 ticks per microsecond.
 *
 * @param us  Time in microseconds (integer or expression).
 * @return    Equivalent tick count as @c uint64_t.
 */
#define UWB_US_TO_TICKS(us) ((uint64_t)((us) * 63898ULL))

/**
 * @brief Master RX window waiting for RESPONSE after POLL (ms).
 *
 * Must exceed worst-case slave processing and TX scheduling time (~2 ms).
 *
 * @todo Reduce to ~20 ms once timing is stable.
 */
#define T_RESPONSE_RX_WAIT  100U    /* ms — conservative for testing */
#define T_FINAL_RX_WAIT  100U
/**
 * @brief Slave RX window covering the entire TWR exchange (ms).
 *
 * Must exceed POLL air time + FINAL delay + FINAL air time + the full
 * passive observer chain.
 *
 * @todo Reduce to ~60 ms once passive chain length is known.
 */
#define T_TWR_RX_WAIT       300U    /* ms — conservative for testing */

/**
 * @brief Delay from RESPONSE reception to FINAL transmission (ticks).
 *
 * Gives the master CPU enough time to decode the RESPONSE, compute
 * timestamps, and schedule the delayed TX.  3 000 µs is safe for a
 * Cortex-M4 running at 64–168 MHz.
 *
 * Equals approximately 191 694 000 ticks.
 *
 * @todo Reduce if CPU budget allows, to lower ranging latency.
 */
#define T_FINAL_TX_ASAP_TICKS  UWB_US_TO_TICKS(800)

/**
 * @brief Minimum delay before a passive observer's TX (ticks).
 *
 * Used when the passive device schedules its own transmission as soon as
 * possible after capturing the full TWR exchange (1 000 µs guard).
 */
#define T_PASSIVE_TX_ASAP_TICKS UWB_US_TO_TICKS(500)

/**
 * @brief Timeout waiting for TXFRS after scheduling the delayed FINAL (ms).
 *
 * Must exceed T_FINAL_TX_DELAY converted to ms plus OS scheduling jitter.
 */
#define T_FINAL_TX_WAIT_MS      10U     /* ms */

/**
 * @brief Timeout waiting for TXFRS after scheduling the passive TX (ms).
 *
 * Must exceed T_PASSIVE_TX_ASAP_TICKS converted to ms plus OS jitter.
 */
#define T_PASSIVE_TX_WAIT_MS    10U     /* ms */

/**
 * @brief Consecutive SYNC timeouts before a slave promotes itself to master.
 *
 * Keep low (2–3) to recover quickly when the master disappears.
 *
 * @todo Increase to 3–5 for larger networks to reduce spurious promotions.
 */
#define SYNC_TIMEOUT_MAX        2U

/**
 * @brief Broadcast destination address.
 *
 * Frames addressed to this ID are accepted by every device regardless
 * of its own ID.
 */
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
    UWB_TWR_RECEIVED_PASSIVE,   /**< Passive observer: full exchange captured; ready for TDoA. */
    UWB_TWR_NOTHING,            /**< Exchange completed but this device was not involved. */
    /* --- Failure --- */
    UWB_TWR_NOT_ENOUGH_DEVICES, /**< No peers available or sync state invalid. */
    UWB_TWR_TX_FAILED,          /**< TX of POLL, RESPONSE, or FINAL failed. */
    UWB_TWR_TIMEOUT,            /**< RX window expired before the expected message arrived. */
    UWB_TWR_UNEXPECTED_MASTER,  /**< Frame received from an unexpected sender. */
} uwb_etwr_result_t;

/* -----------------------------------------------------------------------
 * Public API
 * ----------------------------------------------------------------------- */

/**
 * @brief Perform one synchronisation round.
 *
 * The device either broadcasts a SYNC frame (master role) or listens for
 * one (slave role) and handles join negotiation.
 *
 * @return  @ref uwb_sync_result_t indicating the outcome and the role
 *          this device will hold for the subsequent TWR phase.
 */
uwb_sync_result_t uwb_sync(void);

/**
 * @brief Execute one extended two-way ranging exchange.
 *
 * Drives the full POLL → RESPONSE → FINAL sequence based on the role
 * determined during @ref uwb_sync.
 *
 * @param sync_result  Result from the preceding @ref uwb_sync call;
 *                     determines whether this device acts as initiator,
 *                     responder, or passive observer.
 * @return  @ref uwb_etwr_result_t indicating success or the failure mode.
 */
uwb_etwr_result_t uwb_extended_twr(uwb_sync_result_t sync_result);

/**
 * @brief Execute a simplified TWR exchange for testing purposes.
 *
 * Behaves like @ref uwb_extended_twr but uses a reduced frame sequence
 * suitable for bench testing without a full network.
 *
 * @param sync_result  Result from the preceding @ref uwb_sync call.
 * @return  @ref uwb_etwr_result_t indicating success or the failure mode.
 */
uwb_etwr_result_t uwb_twr_test(uwb_sync_result_t sync_result);

/**
 * @brief Broadcast or receive the ranging result and compute sleep time.
 *
 * After a completed TWR exchange, the master broadcasts timestamps/distances
 * so all nodes hold consistent data.  Returns the duration the caller
 * should sleep before the next cycle.
 *
 * @param etwr_result  Result from the preceding @ref uwb_extended_twr call.
 * @param sleep_time   Nominal sleep duration for this cycle (ms).
 * @return  Actual sleep duration (ms) adjusted for any timing drift.
 */
uint32_t uwb_share(uwb_etwr_result_t etwr_result, uint32_t sleep_time);

#ifdef __cplusplus
}
#endif