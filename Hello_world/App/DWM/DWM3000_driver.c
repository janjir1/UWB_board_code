// ─── Includes ─────────────────────────────────────────────────────────────────

#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "cmsis_os.h"
#include "cmsis_os2.h"
#include "queue.h"
#include "stm32l4xx_hal.h"

#include "dw3000_hw.h"
#include "dw3000_spi.h"
#include "dw3000_deca_regs.h"
#include "deca_device_api.h"
#include "main.h"
#include "../Generic/my_print.h"

#include "DWM3000_setup.h"
#include "DWM3000_driver.h"

extern const struct dwt_probe_s dw3000_probe_interf;
extern SPI_HandleTypeDef hspi1;

// ─── Queues ───────────────────────────────────────────────────────────────────

QueueHandle_t rx_queue     = NULL;
QueueHandle_t tx_queue     = NULL;
QueueHandle_t wakeup_queue = NULL;

// ─── Constants ────────────────────────────────────────────────────────────────

static uint16_t s_short_addr = 0;   // IEEE 802.15.4 short address, derived from MCU UID

// ─── Internal helpers ─────────────────────────────────────────────────────────

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief Convert a 5-byte little-endian DW3000 timestamp buffer to a uint64_t.
 *        DW3000 timestamps are 40-bit values stored LSB-first across 5 bytes.
 *
 * @param buf  Pointer to a 5-byte buffer containing the raw timestamp.
 *
 * @return 40-bit timestamp as uint64_t (upper 24 bits are always zero).
 */
static inline uint64_t ts_buf_to_u64(const uint8_t buf[5])
{
    uint64_t ts = 0;
    for (int i = 0; i < 5; i++)
        ts |= ((uint64_t)buf[i]) << (i * 8);
    return ts;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief Log a human-readable RX error message based on the DW3000 status register bitmask.
 *        Checks the most common error flags in priority order.
 *
 * @param status  32-bit value from dwt_cb_data_t.status, checked against DWT_INT_* masks.
 *
 * @return None.
 */
static void log_rx_error(uint32_t status)
{
    if      (status & DWT_INT_RXPHE_BIT_MASK) mprintf("RX ERR: PHR error\r\n");
    else if (status & DWT_INT_RXFCE_BIT_MASK) mprintf("RX ERR: CRC mismatch\r\n");
    else if (status & DWT_INT_RXFSL_BIT_MASK) mprintf("RX ERR: Frame sync loss\r\n");
    else if (status & DWT_INT_RXSTO_BIT_MASK) mprintf("RX ERR: SFD timeout\r\n");
    else                                       mprintf("RX ERR: unknown 0x%08lX\r\n", status);
}

// ─── Init ─────────────────────────────────────────────────────────────────────

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief Initialise the DW3000 hardware and driver.
 *        Performs GPIO/SPI init, hard reset, driver probe, IDLE_RC poll,
 *        dwt_initialise, and SPI speed-up. Leaves chip in IDLE_PLL with
 *        EXTI interrupt enabled.
 *
 *        Must be called before any other dwm_* function.
 *
 * @return true on success, false if any step fails (probe, IDLE_RC timeout, initialise).
 */
bool dwm_init(void)
{
    HAL_NVIC_DisableIRQ(EXTI0_IRQn);

    dw3000_hw_init();
    dw3000_hw_reset();
    osDelay(10);

    if (dwt_probe((struct dwt_probe_s *)&dw3000_probe_interf) != DWT_SUCCESS) {
        mprintf("dwt_probe failed\r\n");
        return false;
    }

    // Poll until chip reaches IDLE_RC — required before dwt_initialise()
    uint32_t timeout = 500;
    while (!dwt_checkidlerc()) {
        if (--timeout == 0) {
            mprintf("IDLE_RC timeout\r\n");
            return false;
        }
        osDelay(1);
    }

    if (dwt_initialise(DWT_DW_INIT) != DWT_SUCCESS) {
        mprintf("dwt_initialise failed\r\n");
        return false;
    }

    dwt_enablespicrccheck(DWT_SPI_CRC_MODE_NO, NULL);
    dw3000_spi_speed_fast();

    __HAL_GPIO_EXTI_CLEAR_IT(DWM_EXTI_Pin);
    HAL_NVIC_EnableIRQ(EXTI0_IRQn);
    return true;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief Verify basic DW3000 functionality after initialisation.
 *        Checks device ID, OTP crystal trim, and TX path via event counter.
 *
 *        Call after dwm_init() and dwm_configure() have both succeeded.
 *
 * @return true if all checks pass, false on any failure.
 */
bool dwm_selftest(void)
{
    if (dwt_readdevid() != DWT_DEVICE_ID) {
        mprintf("Self-test: bad device ID 0x%08lX\r\n", dwt_readdevid());
        return false;
    }

    // XTAL trim of 0 indicates OTP was never programmed — likely hardware fault
    if (dwt_getxtaltrim() == 0) {
        mprintf("Self-test: XTAL trim not programmed\r\n");
        return false;
    }

    // Fire one frame and verify the TX event counter increments
    dwt_configeventcounters(1);  // reset + enable

    uint8_t tx_msg[] = { 0xC5, 0x00, 0x01, 0x02, 0x03, 0x04 };
    dwt_writetxdata(sizeof(tx_msg), tx_msg, 0);
    dwt_writetxfctrl(sizeof(tx_msg) + DWM_CRC_LEN, 0, 0);
    dwt_starttx(DWT_START_TX_IMMEDIATE);
    osDelay(2);  // 6.8Mbps frame completes in <1ms — 2ms provides safe margin

    dwt_deviceentcnts_t cnt;
    dwt_readeventcounters(&cnt);
    if (cnt.TXF < 1) {
        mprintf("Self-test: TX frame counter did not increment\r\n");
        return false;
    }

    return true;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief Restore runtime-only settings that are not saved to AON and are lost on sleep/reset.
 *        Must be called both at startup (via dwm_configure) and after every wakeup (via dwm_wakeup).
 *        Settings restored: RX/TX antenna delays, PAN ID, short address, interrupt mask.
 *
 * @param addr  IEEE 802.15.4 16-bit short address to assign to the device.
 *
 * @return None.
 */
void dwm_restore_runtime(uint16_t addr)
{
    dwt_setrxantennadelay(DWM_ANT_DELAY);
    dwt_settxantennadelay(DWM_ANT_DELAY);
    dwt_setpanid(DWM_PAN_ID);
    dwt_setaddress16(addr);
    dwt_setinterrupt(DWM_IRQ_MASK, 0, DWT_ENABLE_INT);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief Create the three FreeRTOS queues used for TX done, RX frames, and wakeup events,
 *        and register the DW3000 ISR callbacks.
 *        Called once during dwm_configure(). Queues are global and used by all dwm_tx/rx functions.
 *
 * @return true on success, false if any queue allocation fails.
 */
bool dwm_init_queues(void)
{
    rx_queue     = xQueueCreate(4, sizeof(dwm_rx_raw_frame_t));
    tx_queue     = xQueueCreate(2, sizeof(uint64_t));
    wakeup_queue = xQueueCreate(1, sizeof(uint8_t));

    if (!rx_queue || !tx_queue || !wakeup_queue) {
        mprintf("Queue creation failed\r\n");
        return false;
    }

    static dwt_callbacks_s callbacks = {
        .cbTxDone = cb_tx_done,
        .cbRxOk   = cb_rx_ok,
        .cbRxTo   = cb_rx_to,
        .cbRxErr  = cb_rx_err,
        .cbSPIErr = NULL,
        .cbSPIRdy = cb_spi_rdy,
    };
    dwt_setcallbacks(&callbacks);
    return true;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief Configure the DW3000 UWB radio and TX RF parameters, derive and assign the short address,
 *        initialise queues, and apply runtime settings.
 *        The short address is derived from the MCU unique ID (XOR of UID word 0 and word 1).
 *
 * @return true on success, false if dwt_configure or queue init fails.
 */
bool dwm_configure(void)
{
    static const dwt_config_t config = {
        .chan           = DWM_UWB_CHANNEL,
        .txPreambLength = DWM_UWB_PREAMBLE_LEN,
        .rxPAC          = DWM_UWB_PAC,
        .txCode         = DWM_UWB_TX_CODE,
        .rxCode         = DWM_UWB_RX_CODE,
        .sfdType        = DWM_UWB_SFD_TYPE,
        .dataRate       = DWM_UWB_DATA_RATE,
        .phrMode        = DWM_UWB_PHR_MODE,
        .phrRate        = DWM_UWB_PHR_RATE,
        .sfdTO          = DWM_UWB_SFD_TO,
        .stsMode        = DWM_UWB_STS_MODE,
        .stsLength      = DWM_UWB_STS_LENGTH,
        .pdoaMode       = DWM_UWB_PDOA_MODE,
    };

    static const dwt_txconfig_t tx_config = {
        .PGdly   = TX_RF_PGdly,
        .power   = TX_RF_Power,
        .PGcount = TX_RF_PGcount,
    };

    dwt_forcetrxoff();

    if (dwt_configure((dwt_config_t *)&config) != DWT_SUCCESS) {
        mprintf("dwt_configure failed\r\n");
        return false;
    }
    dwt_configuretxrf((dwt_txconfig_t *)&tx_config);

    s_short_addr = (uint16_t)((HAL_GetUIDw0() ^ HAL_GetUIDw1()) & 0xFFFF);
    mprintf("Short addr: 0x%04X\r\n", s_short_addr);

    if (!dwm_init_queues()) return false;

    dwm_restore_runtime(s_short_addr);
    return true;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief Return the IEEE 802.15.4 short address assigned to this device.
 *        Valid only after dwm_configure() has been called successfully.
 *
 * @return 16-bit short address, or 0 if dwm_configure has not been called.
 */
uint16_t dwm_get_addr(void)
{
    return s_short_addr;
}

// ─── ISR Callbacks ────────────────────────────────────────────────────────────

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief DW3000 RX complete callback — fired from ISR context on successful frame reception.
 *        Reads frame data, RX timestamp, and diagnostic accumulator, then posts to rx_queue.
 *
 * @param cb_data  Pointer to callback data provided by the DW3000 driver (length, status).
 *
 * @return None.
 */
void cb_rx_ok(const dwt_cb_data_t *cb_data)
{
    dwm_rx_raw_frame_t frame = { .type = DWM_RX_OK, .status = cb_data->status };

    frame.len = cb_data->datalength - DWM_CRC_LEN;
    dwt_readrxdata(frame.data, frame.len, 0);
    dwt_readrxtimestamp(frame.ts_buf, DWT_COMPAT_NONE);
    dwt_readdiagnostics_acc(&frame.rx_diag, DWT_ACC_IDX_IP_M);

    BaseType_t woken = pdFALSE;
    xQueueSendFromISR(rx_queue, &frame, &woken);
    portYIELD_FROM_ISR(woken);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief DW3000 RX error callback — fired from ISR context when a frame is received with errors
 *        (CRC mismatch, PHR error, frame sync loss, SFD timeout, etc.).
 *        Posts a DWM_RX_ERR frame with the raw status bitmask to rx_queue.
 *
 * @param cb_data  Pointer to callback data provided by the DW3000 driver (status bitmask).
 *
 * @return None.
 */
void cb_rx_err(const dwt_cb_data_t *cb_data)
{
    dwm_rx_raw_frame_t frame = { .type = DWM_RX_ERR, .status = cb_data->status };

    BaseType_t woken = pdFALSE;
    xQueueSendFromISR(rx_queue, &frame, &woken);
    portYIELD_FROM_ISR(woken);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief DW3000 RX timeout callback — fired from ISR context when the RX timeout expires
 *        without a frame being received. Posts a DWM_RX_TIMEOUT frame to rx_queue.
 *
 * @param cb_data  Unused — timeout carries no additional data.
 *
 * @return None.
 */
void cb_rx_to(const dwt_cb_data_t *cb_data)
{
    (void)cb_data;
    dwm_rx_raw_frame_t frame = { .type = DWM_RX_TIMEOUT, .status = 0 };

    BaseType_t woken = pdFALSE;
    xQueueSendFromISR(rx_queue, &frame, &woken);
    portYIELD_FROM_ISR(woken);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief DW3000 TX complete callback — fired from ISR context when a frame has been fully transmitted.
 *        Reads the TX timestamp (RMARKER, adjusted by programmed antenna delay) and posts it to tx_queue.
 *
 * @param cb_data  Unused — TX done carries no additional data beyond the implicit timestamp.
 *
 * @return None.
 */
void cb_tx_done(const dwt_cb_data_t *cb_data)
{
    (void)cb_data;
    uint8_t ts_buf[5] = {0};
    dwt_readtxtimestamp(ts_buf);
    uint64_t ts_64 = ts_buf_to_u64(ts_buf);

    BaseType_t woken = pdFALSE;
    xQueueSendFromISR(tx_queue, &ts_64, &woken);
    portYIELD_FROM_ISR(woken);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief DW3000 SPI ready callback — fired from ISR context when the chip asserts SPIRDY after wakeup.
 *        Posts a single byte to wakeup_queue to unblock dwm_wakeup().
 *
 * @param cb_data  Unused.
 *
 * @return None.
 */
void cb_spi_rdy(const dwt_cb_data_t *cb_data)
{
    (void)cb_data;
    uint8_t rdy = 1;

    BaseType_t woken = pdFALSE;
    xQueueSendFromISR(wakeup_queue, &rdy, &woken);
    portYIELD_FROM_ISR(woken);
}

// ─── RX ───────────────────────────────────────────────────────────────────────

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief Arm the DW3000 receiver and block until a frame is received, a timeout occurs, or an error is reported.
 *        On DWM_RX_OK, result is fully populated including timestamp, RSSI, and first-path power.
 *        On DWM_RX_ERR or DWM_RX_TIMEOUT, only result->type and result->status are set.
 *
 * @param result      Pointer to caller-allocated dwm_rx_frame_t to be populated.
 * @param timeout_ms  RX timeout in milliseconds. The DW3000 will always fire a timeout event,
 *                    so this function always returns — it never blocks indefinitely.
 *
 * @return None. Check result->type for DWM_RX_OK / DWM_RX_ERR / DWM_RX_TIMEOUT.
 */
void dwm_rx(dwm_rx_frame_t *result, uint32_t timeout_ms)
{
    dwt_setrxtimeout(timeout_ms * 1000U);
    dwt_setpreambledetecttimeout(0);
    dwt_rxenable(DWT_START_RX_IMMEDIATE);

    dwm_rx_raw_frame_t frame;
    xQueueReceive(rx_queue, &frame, portMAX_DELAY);
    dwt_forcetrxoff();

    result->type   = frame.type;
    result->status = frame.status;

    if (frame.type == DWM_RX_ERR) { log_rx_error(frame.status); return; }
    if (frame.type != DWM_RX_OK)  { return; }

    result->len          = frame.len;
    result->rx_timestamp = ts_buf_to_u64(frame.ts_buf);
    memcpy(result->data, frame.data, frame.len);

    if (dwt_calculate_rssi(&frame.rx_diag, DWT_ACC_IDX_IP_M, &result->rssi_q8) != DWT_SUCCESS)
        mprintf("RSSI calc failed\r\n");

    if (dwt_calculate_first_path_power(&frame.rx_diag, DWT_ACC_IDX_IP_M, &result->fp_q8) != DWT_SUCCESS)
        mprintf("FP power calc failed\r\n");
}

// ─── TX ───────────────────────────────────────────────────────────────────────

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief Transmit a frame immediately and block until the TX done callback posts the timestamp.
 *        Forces TRX off before transmitting to ensure a clean state.
 *        On success, frame->tx_timestamp is updated with the RMARKER timestamp of the transmitted frame.
 *
 * @param frame  Pointer to a dwm_tx_frame_t containing:
 *               - data: pointer to payload buffer (must remain valid until function returns)
 *               - len:  payload length in bytes (excluding the 2-byte CRC added by hardware)
 *               - tx_timestamp: populated on return with the 40-bit TX RMARKER timestamp
 *
 * @return DWM_TX_OK     if the frame was transmitted and timestamp received successfully.
 *         DWM_TX_LATE   if dwt_starttx() returned DWT_ERROR (delayed TX deadline missed).
 *         DWM_TX_ERROR  if the TX done callback did not fire within 10ms.
 */
dwm_tx_event_type_t dwm_tx(dwm_tx_frame_t *frame)
{
    dwt_forcetrxoff();

    dwt_writetxdata(frame->len + DWM_CRC_LEN, frame->data, 0);
    dwt_writetxfctrl(frame->len + DWM_CRC_LEN, 0, 1);

    xQueueReset(tx_queue);

    if (dwt_starttx(DWT_START_TX_IMMEDIATE) == DWT_ERROR)
        return DWM_TX_LATE;

    uint64_t ts = 0;
    if (xQueueReceive(tx_queue, &ts, pdMS_TO_TICKS(10)) != pdTRUE)
        return DWM_TX_ERROR;

    frame->tx_timestamp = ts;
    return DWM_TX_OK;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief Transmit a frame at a programmed delayed RMARKER time and block until the TX done callback
 *        posts the actual TX timestamp.
 *        Forces TRX off before transmitting to ensure a clean state.
 *        On success, frame->tx_timestamp is updated with the RMARKER timestamp of the transmitted frame.
 *
 * @param frame             Pointer to a dwm_tx_frame_t containing:
 *                          - data: pointer to payload buffer (must remain valid until function returns)
 *                          - len:  payload length in bytes (excluding the 2-byte CRC added by hardware)
 *                          - tx_timestamp: populated on return with the 40-bit TX RMARKER timestamp
 *
 * @param final_rmarker_ts  Desired final 40-bit TX RMARKER timestamp, including antenna delay.
 *
 * @return DWM_TX_OK     if the frame was transmitted and timestamp received successfully.
 *         DWM_TX_LATE   if dwt_starttx() returned DWT_ERROR (delayed TX deadline missed).
 *         DWM_TX_ERROR  if the TX done callback did not fire within 10ms.
 */
dwm_tx_event_type_t dwm_tx_delayed(dwm_tx_frame_t *frame, uint64_t final_rmarker_ts, uint32_t wait_ms)
{
    const uint64_t TS40_MASK = 0xFFFFFFFFFFULL;

    dwt_forcetrxoff();

    dwt_writetxdata(frame->len + DWM_CRC_LEN, frame->data, 0);
    dwt_writetxfctrl(frame->len + DWM_CRC_LEN, 0, 1);

    uint16_t tx_ant_delay = dwt_gettxantennadelay();

    // dwt_setdelayedtrxtime() wants raw TX time (without antenna delay),
    // encoded as the high 32 bits of the 40-bit timestamp.
    uint64_t raw_tx_time_40 = (final_rmarker_ts - tx_ant_delay) & TS40_MASK;
    uint32_t delayed_time   = (uint32_t)(raw_tx_time_40 >> 8);

    dwt_setdelayedtrxtime(delayed_time);

    xQueueReset(tx_queue);

    if (dwt_starttx(DWT_START_TX_DELAYED) == DWT_ERROR)
        return DWM_TX_LATE;

    uint64_t ts = 0;
    if (xQueueReceive(tx_queue, &ts, pdMS_TO_TICKS(wait_ms)) != pdTRUE)
        return DWM_TX_ERROR;

    frame->tx_timestamp = ts;
    return DWM_TX_OK;
}

// ─── Sleep / Wakeup ───────────────────────────────────────────────────────────

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief Put the DW3000 into DEEPSLEEP, saving UWB config to AON.
 *        The chip will wake on assertion of the WAKEUP pin (via dwm_wakeup).
 *        PGF calibration will run automatically on the next wakeup.
 *
 *        NOTE: Must only be called when the chip is in IDLE_PLL state,
 *        i.e. after dwm_configure() or after a prior dwm_wakeup() has completed.
 *
 * @return None.
 */
void dwm_sleep(void)
{
    dwt_configuresleep(
        DWT_CONFIG | DWT_PGFCAL,    // save UWB config to AON; run PGF cal on wakeup
        DWT_WAKE_WUP | DWT_SLP_EN   // wake on WAKEUP pin; enable sleep
    );
    dwt_entersleep(DWT_DW_IDLE);    // chip is in IDLE_PLL — must match actual state
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief Wake the DW3000 from DEEPSLEEP by asserting the WAKEUP pin, then restore all runtime settings.
 *        Blocks until SPIRDY is received (max 10ms), then calls dwt_restoreconfig(1) to reload
 *        UWB config and run PGF calibration. Restores antenna delays, PAN ID, address, and interrupts.
 *
 *        After this function returns the chip is in IDLE_PLL and ready for TX/RX.
 *
 * @return None. Prints a warning via mprintf if SPIRDY is not received within 10ms.
 */
void dwm_wakeup(void)
{
    xQueueReset(wakeup_queue);

    HAL_GPIO_WritePin(DWM_WAKEUP_GPIO_Port, DWM_WAKEUP_Pin, GPIO_PIN_SET);

    uint8_t rdy = 0;
    if (xQueueReceive(wakeup_queue, &rdy, pdMS_TO_TICKS(10)) != pdTRUE)
        mprintf("DW3000 wakeup timeout!\r\n");

    osDelay(1);
    HAL_GPIO_WritePin(DWM_WAKEUP_GPIO_Port, DWM_WAKEUP_Pin, GPIO_PIN_RESET);

    HAL_NVIC_DisableIRQ(EXTI0_IRQn);
    __HAL_GPIO_EXTI_CLEAR_IT(DWM_EXTI_Pin);
    dwt_restoreconfig(1);
    HAL_NVIC_EnableIRQ(EXTI0_IRQn);
    osDelay(2);  // PLL stabilisation — minimum safe margin after restoreconfig(1)

    dwm_restore_runtime(s_short_addr);
}

// ─── Test: continuous TX/RX with sleep ───────────────────────────────────────────

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief Test function — transmit a counter frame every ~1s with sleep between transmissions.
 *        Each frame embeds the counter byte and the previous TX timestamp.
 *        Prints the TX timestamp and result for each frame over serial.
 *
 *        Frame layout: [0xDE][0xAD][counter:1][prev_ts:5]["HELLOWORLD":10]
 *
 *        NOTE: This function never returns. Intended for standalone testing only.
 *
 * @return None.
 */
void dwm_tx_continuous(void)
{
    uint8_t msg[18];
    msg[0] = 0xDE;
    msg[1] = 0xAD;
    memcpy(&msg[8], "HELLOWORLD", 10);

    dwm_tx_frame_t frame = { .data = msg, .len = sizeof(msg), .tx_timestamp = 0 };
    uint8_t counter = 0;

    while (true) {
        msg[2] = counter;
        memcpy(&msg[3], &frame.tx_timestamp, 5);

        dwm_wakeup();
        dwm_tx_event_type_t result = dwm_tx(&frame);
        dwm_sleep();

        switch (result) {
        case DWM_TX_OK:
            mprintf("[%3d] Sent TS: %08lX%02X\r\n",
                counter,
                (uint32_t)(frame.tx_timestamp >> 8),
                (uint8_t) (frame.tx_timestamp & 0xFF));
            break;
        case DWM_TX_LATE:  mprintf("[%3d] TX LATE\r\n",  counter); break;
        case DWM_TX_ERROR: mprintf("[%3d] TX ERROR\r\n", counter); break;
        }

        counter++;
        osDelay(1000);
    }
}

#define DWM_TX_BASE_DTU   0x0010C95801ULL
#define DWM_TX_DELAY_DTU  0x2F9B80000ULL

void dwm_tx_continuous_delayed(void)

{
    uint8_t msg[18];
    msg[0] = 0xDE;
    msg[1] = 0xAD;
    memcpy(&msg[8], "HELLOWORLD", 10);

    dwm_tx_frame_t frame  = { .data = msg, .len = sizeof(msg), .tx_timestamp = 0 };
    uint8_t        counter = 0;

    while (true) {
        msg[2] = counter;
        memcpy(&msg[3], &frame.tx_timestamp, 5);

        dwm_wakeup();
        dwm_tx_event_type_t result = dwm_tx_delayed(&frame, DWM_TX_BASE_DTU + DWM_TX_DELAY_DTU, 1000);
        dwm_sleep();

        switch (result) {
        case DWM_TX_OK:
            mprintf("[%3d] Expected TS: %08lX%02X\r\n",
                counter,
                (uint32_t)((DWM_TX_BASE_DTU + DWM_TX_DELAY_DTU) >> 8),
                (uint8_t) ((DWM_TX_BASE_DTU + DWM_TX_DELAY_DTU) & 0xFF));
            mprintf("[%3d] Sent TS: %08lX%02X\r\n",
                counter,
                (uint32_t)(frame.tx_timestamp >> 8),
                (uint8_t) (frame.tx_timestamp & 0xFF));
            counter++;
            break;
        case DWM_TX_LATE:  mprintf("[%3d] TX LATE\r\n",  counter); break;
        case DWM_TX_ERROR: mprintf("[%3d] TX ERROR\r\n", counter); break;
        }

        osDelay(800);
    }
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief Receive UWB frames continuously, sleeping between listen windows to save power.
 *
 * Sleeps 800ms then listens for 400ms (1200ms cycle). Designed for a ~1000ms TX interval —
 * the 400ms window self-synchronises within 5 cycles (~6s) without explicit timing logic.
 *
 * Prints payload (ASCII), error status, or timeout over serial for each result.
 *
 * @note Never returns. Requires dwm_init() and dwm_configure() to be called first.
 * @note Listen window must not exceed ~1075ms (20-bit DW3000 FWTO hardware limit).
 *
 * @return None.
 */

void dwm_rx_continuous_sleep(void)
{
    dwm_rx_frame_t result = {0};

    while (true) {
        osDelay(800);

        dwm_wakeup();
        dwm_rx(&result, 400);
        dwm_sleep();

        switch (result.type) {
        case DWM_RX_OK:
            mprintf("RX OK %u bytes: ", result.len);
                    for (uint16_t i = 0; i < result.len; i++) {
                        mprintf("%c", (result.data[i] >= 32 && result.data[i] < 127)
                                    ? result.data[i] : '.');
                }
                mprintf("\r\n", result.status);
            
            break;
        case DWM_RX_ERR:
            mprintf("RX ERR: 0x%08lX\r\n", result.status);
            break;
        case DWM_RX_TIMEOUT:
            mprintf("Timeout\r\n");
            break;  // silent — expected during sync phase
        }
    }
}
