#include "uart.h"
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <inttypes.h>
#include <stdlib.h>
#include "../UWB_app/uwb_network.h"
#include "my_print.h"
#include "power.h"

/* ============================================================================
 * Boot-phase RX state
 * ============================================================================ */

static uint8_t          uart_boot_rx_buf[UART_BOOT_RX_LEN];
static volatile uint16_t uart_boot_rx_len   = 0;
static volatile bool     uart_boot_rx_done  = false;
static volatile bool     uart_boot_rx_error = false;

/* ============================================================================
 * Runtime RST-phase RX state
 * ============================================================================ */

/** @brief Single-byte sentinel that triggers a soft reset when received. */
#define UART_RST_BYTE     0x03U
/** @brief RST receive buffer size (only 1 byte used; extra headroom for DMA). */
#define UART_RST_BUF_SIZE 8U

static uint8_t           uart_rst_rx_buf[UART_RST_BUF_SIZE];
static volatile uint16_t uart_rst_rx_len  = 0;
static volatile bool     uart_rst_rx_done = false;

/** @brief true while the UART DMA is armed for the RST phase; false during boot phase. */
static volatile bool uart_rst_phase = false;

/* ============================================================================
 * Boot defaults & helpers
 * ============================================================================ */

/**
 * @brief Fallback EKF position hints used when no valid UART configuration is received.
 *
 * Positions are in metres relative to Anchor 0 (world origin).
 * These match the physical anchor layout used during development and must be
 * updated when the anchor placement changes.
 */
static const ekf_node_hint_t ekf_pos_hints_default[4] = {
    { 0x63D8u,  0.0f,  0.0f,  0.000f },
    { 0x91EDu, -2.0f,  2.0f,  0.586f },
    { 0xC019u,  1.0f,  2.0f, -0.447f },
    { 0x28CCu, -3.0f, -1.0f, -0.064f },
};

/**
 * @brief Populate @p cfg with compile-time default values.
 *
 * Sets @c charge_only = false, @c valid = false, and copies
 * @c ekf_pos_hints_default into @c cfg->hints.
 *
 * @param[out] cfg  Config struct to initialise.
 */
static void boot_config_load_defaults(boot_config_t *cfg)
{
    cfg->charge_only = false;
    cfg->valid       = false;
    memcpy(cfg->hints, ekf_pos_hints_default, sizeof(cfg->hints));
}

/**
 * @brief Parse a semicolon-delimited boot configuration string into @p cfg.
 *
 * Expected format:
 * @code
 *   RUN=<0|1>;<hex_id>,<x>,<y>,<z>;<hex_id>,<x>,<y>,<z>;...
 * @endcode
 * Exactly four node hint tokens must follow the @c RUN field.
 * @c cfg is pre-populated with defaults via @ref boot_config_load_defaults
 * before parsing, so partial parses leave @c cfg->valid = false.
 *
 * @param msg      Pointer to the received character data (not NUL-terminated).
 * @param msg_len  Number of valid bytes in @p msg.
 * @param[out] cfg Config struct to populate.
 * @return true on successful parse, false on any format error.
 */
static bool boot_config_decode(const char *msg, uint16_t msg_len, boot_config_t *cfg)
{
    char         buf[UART_BOOT_RX_LEN + 1];
    char        *tok;
    unsigned int run_flag = 0;
    unsigned int addr     = 0;
    int          x = 0, y = 0, z = 0;

    boot_config_load_defaults(cfg);

    if (msg_len >= UART_BOOT_RX_LEN)
        msg_len = UART_BOOT_RX_LEN - 1;

    memcpy(buf, msg, msg_len);
    buf[msg_len] = '\0';

    tok = strtok(buf, ";");
    if (tok == NULL || strncmp(tok, "RUN=", 4) != 0) {
        mprintf("[BOOTCFG] bad RUN token\r\n");
        return false;
    }

    run_flag         = (unsigned int)strtoul(tok + 4, NULL, 10);
    cfg->charge_only = (run_flag == 0u);

    for (int i = 0; i < 4; i++) {
        tok = strtok(NULL, ";");
        if (tok == NULL) {
            mprintf("[BOOTCFG] missing token[%d]\r\n", i);
            return false;
        }

        if (sscanf(tok, "%x,%d,%d,%d", &addr, &x, &y, &z) != 4) {
            mprintf("[BOOTCFG] parse failed at token[%d]='%s'\r\n", i, tok);
            return false;
        }

        cfg->hints[i].id = (uint16_t)addr;
        cfg->hints[i].x  = (float)x;
        cfg->hints[i].y  = (float)y;
        cfg->hints[i].z  = (float)z;
    }

    cfg->valid = true;
    return true;
}

/* ============================================================================
 * Boot-phase API
 * ============================================================================ */

/**
 * @brief Transmit this node's ID and arm the UART DMA to receive a boot config.
 *
 * Transmits the 4-hex-digit node ID followed by @c \\r\\n over LPUART1, then
 * starts a @c HAL_UARTEx_ReceiveToIdle_DMA transfer into @c uart_boot_rx_buf.
 * The half-transfer DMA interrupt is disabled to avoid a spurious early callback.
 *
 * Must be called once during the boot sequence before @ref uart_boot_config_read.
 *
 * @return true if both the transmit and DMA arm succeeded, false otherwise.
 */
bool uart_boot_start(void)
{
    uint16_t my_id = network_get_ownid();
    char     tx_buf[16];

    uart_rst_phase    = false;
    uart_boot_rx_done  = false;
    uart_boot_rx_error = false;
    uart_boot_rx_len   = 0;
    memset(uart_boot_rx_buf, 0, sizeof(uart_boot_rx_buf));

    int tx_len = snprintf(tx_buf, sizeof(tx_buf), "%04X\r\n", my_id);

    if (HAL_UART_Transmit(&hlpuart1, (uint8_t *)tx_buf, (uint16_t)tx_len, 100) != HAL_OK)
        return false;

    if (HAL_UARTEx_ReceiveToIdle_DMA(&hlpuart1, uart_boot_rx_buf, UART_BOOT_RX_LEN) != HAL_OK)
        return false;

    if (hlpuart1.hdmarx != NULL)
        __HAL_DMA_DISABLE_IT(hlpuart1.hdmarx, DMA_IT_HT);

    return true;
}

/**
 * @brief Read and decode the boot configuration received over UART.
 *
 * Must be called after a sufficient timeout following @ref uart_boot_start.
 * Checks that a complete, error-free frame was received, then decodes it via
 * @ref boot_config_decode. Falls back to defaults if the receive failed or
 * the frame could not be parsed.
 *
 * @return Decoded @c boot_config_t; @c valid is true on success, false if
 *         no frame was received or decoding failed (defaults still populated).
 */
boot_config_t uart_boot_config_read(void)
{
    boot_config_t boot_cfg;
    boot_config_load_defaults(&boot_cfg);

    /* TODO(BUG): Early return bypasses all receive validation and decode logic
     * below. Remove these two lines when UART boot config reception is ready
     * to be re-enabled. */
    boot_cfg.valid = true;
    return boot_cfg;

    if (!uart_boot_rx_done || uart_boot_rx_error || uart_boot_rx_len == 0) {
        mprintf("[BOOT] no valid rx (done=%d err=%d len=%u)\r\n",
                uart_boot_rx_done  ? 1 : 0,
                uart_boot_rx_error ? 1 : 0,
                uart_boot_rx_len);
        boot_cfg.valid = false;
        return boot_cfg;
    }

    HAL_UART_AbortReceive(&hlpuart1);

    uint16_t rx_len = uart_boot_rx_len;
    if (rx_len >= UART_BOOT_RX_LEN)
        rx_len = UART_BOOT_RX_LEN - 1;
    uart_boot_rx_buf[rx_len] = '\0';

    bool ok = boot_config_decode((const char *)uart_boot_rx_buf, rx_len, &boot_cfg);
    mprintf("[BOOT] decode %s\r\n", ok ? "OK" : "FAILED");

    return boot_cfg;
}

/* ============================================================================
 * Runtime RST-phase API
 * ============================================================================ */

/**
 * @brief Arm the UART DMA to listen for a single-byte RST command.
 *
 * No-ops if a byte has already been received and is pending processing
 * (@c uart_rst_rx_done is true). Aborts any active receive before
 * re-arming. The half-transfer interrupt is disabled.
 *
 * Should be called once after initialisation and then re-called by
 * @ref uart_periodic_poll after each processed RST event.
 */
void uart_rst_arm(void)
{
    if (uart_rst_rx_done)
        return;

    HAL_UART_AbortReceive(&hlpuart1);
    uart_rst_phase = true;

    if (HAL_UARTEx_ReceiveToIdle_DMA(&hlpuart1, uart_rst_rx_buf, 1) != HAL_OK) {
        mprintf("[RST] arm failed\r\n");
        uart_rst_phase = false;
        return;
    }

    if (hlpuart1.hdmarx != NULL)
        __HAL_DMA_DISABLE_IT(hlpuart1.hdmarx, DMA_IT_HT);
}

/**
 * @brief Transmit current node positions and check for an RST command.
 *
 * Should be called from the main application loop. On each call:
 * 1. Re-arms the RST listener via @ref uart_rst_arm.
 * 2. Transmits @c [POS] lines for @c net->self and all peers (if any peers
 *    are present).
 * 3. If @c UART_RST_BYTE was received since the last call, acknowledges it,
 *    clears the flag, and returns true.
 *
 * @return true if a reset was requested, false otherwise.
 */
bool uart_periodic_poll(void)
{
    const network_t *net = network_get_network();

    uart_rst_arm();

    char tx_buf[48];
    int  tx_len;

    if (net->count != 0) {
        tx_len = snprintf(tx_buf, sizeof(tx_buf), "[POS] 0x%04X %.2f %.2f %.2f\r\n",
                          net->self.id,
                          (double)net->self.pos[0],
                          (double)net->self.pos[1],
                          (double)net->self.pos[2]);
        HAL_UART_Transmit(&hlpuart1, (uint8_t *)tx_buf, (uint16_t)tx_len, 50);

        for (int i = 0; i < (int)net->count; i++) {
            tx_len = snprintf(tx_buf, sizeof(tx_buf), "[POS] 0x%04X %.2f %.2f %.2f\r\n",
                              net->peers[i].id,
                              (double)net->peers[i].pos[0],
                              (double)net->peers[i].pos[1],
                              (double)net->peers[i].pos[2]);
            HAL_UART_Transmit(&hlpuart1, (uint8_t *)tx_buf, (uint16_t)tx_len, 50);
        }
    }

    if (!uart_rst_rx_done)
        return false;

    bool rst        = (uart_rst_rx_buf[0] == UART_RST_BYTE);
    uart_rst_rx_done = false; /* clear flag — arm() will restart on next call */

    if (rst) {
        mprintf("[RST] reset requested\r\n");
        HAL_UART_Transmit(&hlpuart1, (uint8_t *)"RST-ACK\r\n", 9, 50);
    }

    return rst;
}

/**
 * @brief Transmit current battery / power status over LPUART1.
 *
 * Formats and sends a single @c [PWR] line showing voltage, USB connection
 * state, charge-active flag, and charge-complete flag.
 */
void uart_print_power(void)
{
    const BatteryStatus_t *s = power_get_status();
    char tx_buf[48];

    int tx_len = snprintf(tx_buf, sizeof(tx_buf),
                          "[PWR] %.2fV USB:%d CHG:%d STBY:%d\r\n",
                          (double)s->battery_voltage_V,
                          (int)s->usb_connected,
                          (int)s->is_charging,
                          (int)s->charge_complete);

    HAL_UART_Transmit(&hlpuart1, (uint8_t *)tx_buf, (uint16_t)tx_len, 50);
}

/* ============================================================================
 * HAL UART callbacks (ISR context)
 * ============================================================================ */

/**
 * @brief LPUART1 Rx-event callback — fired by DMA idle-line or transfer-complete.
 *
 * Dispatches to the active receive phase:
 * - **RST phase** (@c uart_rst_phase == true): records @p Size and sets
 *   @c uart_rst_rx_done; clears @c uart_rst_phase.
 * - **Boot phase**: records @p Size and sets @c uart_boot_rx_done.
 *
 * Ignores events from any UART handle other than @c hlpuart1.
 *
 * @param huart  UART handle that triggered the event.
 * @param Size   Number of bytes received into the DMA buffer.
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart != &hlpuart1)
        return;

    if (uart_rst_phase) {
        uart_rst_rx_len  = Size;
        uart_rst_rx_done = true;
        uart_rst_phase   = false;
    } else {
        uart_boot_rx_len  = Size;
        uart_boot_rx_done = true;
    }
}

/**
 * @brief LPUART1 error callback — records any UART error during boot receive.
 *
 * Sets @c uart_boot_rx_error so @ref uart_boot_config_read can detect and
 * report the failure instead of trying to decode a corrupt buffer.
 *
 * @param huart  UART handle that encountered the error.
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart == &hlpuart1)
        uart_boot_rx_error = true;
}