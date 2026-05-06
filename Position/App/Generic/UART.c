#include "uart.h"
#include <string.h>
#include <stdio.h>
#include <inttypes.h>
#include <stdlib.h>
#include "../UWB_app/uwb_network.h"
#include "my_print.h"

static uint8_t uart_boot_rx_buf[UART_BOOT_RX_LEN];
static volatile uint16_t uart_boot_rx_len = 0;
static volatile bool uart_boot_rx_done = false;
static volatile bool uart_boot_rx_error = false;

static const ekf_node_hint_t ekf_pos_hints_default[4] = {
    { 0x63D8u,  0.0f,  0.0f,  0.0f },
    { 0x91EDu,  1.0f,  1.0f,  2.0f },
    { 0xC019u, -2.0f,  4.0f, -0.20f },
    { 0x28CCu, -3.0f,  4.0f,  2.5f },
};

static void boot_config_load_defaults(boot_config_t *cfg)
{
    cfg->charge_only = false;
    cfg->valid = false;
    memcpy(cfg->hints, ekf_pos_hints_default, sizeof(cfg->hints));
}

static bool boot_config_decode(const char *msg, uint16_t msg_len, boot_config_t *cfg)
{
    char buf[UART_BOOT_RX_LEN + 1];
    char *tok;
    unsigned int run_flag = 0;
    unsigned int addr = 0;
    int x = 0, y = 0, z = 0;

    boot_config_load_defaults(cfg);

    if (msg_len >= UART_BOOT_RX_LEN) {
        msg_len = UART_BOOT_RX_LEN - 1;
    }
    memcpy(buf, msg, msg_len);
    buf[msg_len] = '\0';

    tok = strtok(buf, ";");
    if (tok == NULL || strncmp(tok, "RUN=", 4) != 0) {
        mprintf("[BOOTCFG] bad RUN token\r\n");
        return false;
    }

    run_flag = (unsigned int)strtoul(tok + 4, NULL, 10);
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
        cfg->hints[i].x = (float)x;
        cfg->hints[i].y = (float)y;
        cfg->hints[i].z = (float)z;
    }

    cfg->valid = true;
    return true;
}

bool uart_boot_start(void)
{
    uint16_t my_id = network_get_ownid();
    char tx_buf[16];
    int tx_len;

    uart_boot_rx_done = false;
    uart_boot_rx_error = false;
    uart_boot_rx_len = 0;
    memset(uart_boot_rx_buf, 0, sizeof(uart_boot_rx_buf));

    tx_len = snprintf(tx_buf, sizeof(tx_buf), "%04X\r\n", my_id);

    if (HAL_UART_Transmit(&hlpuart1, (uint8_t *)tx_buf, (uint16_t)tx_len, 100) != HAL_OK) {
        return false;
    }

    if (HAL_UARTEx_ReceiveToIdle_DMA(&hlpuart1, uart_boot_rx_buf, UART_BOOT_RX_LEN) != HAL_OK) {
        return false;
    }

    if (hlpuart1.hdmarx != NULL) {
        __HAL_DMA_DISABLE_IT(hlpuart1.hdmarx, DMA_IT_HT);
    }

    return true;
}

boot_config_t uart_boot_config_read(void)
{
    boot_config_t boot_cfg;
    uint16_t rx_len;

    boot_config_load_defaults(&boot_cfg);

    if (!uart_boot_rx_done || uart_boot_rx_error || uart_boot_rx_len == 0) {
        mprintf("[BOOT] no valid rx (done=%d err=%d len=%u)\r\n",
                uart_boot_rx_done ? 1 : 0,
                uart_boot_rx_error ? 1 : 0,
                uart_boot_rx_len);
        boot_cfg.valid = false;
        return boot_cfg;
    }

    HAL_UART_AbortReceive(&hlpuart1);

    rx_len = uart_boot_rx_len;
    if (rx_len >= UART_BOOT_RX_LEN) {
        rx_len = UART_BOOT_RX_LEN - 1;
    }
    uart_boot_rx_buf[rx_len] = '\0';

    bool ok = boot_config_decode((const char *)uart_boot_rx_buf, rx_len, &boot_cfg);
    mprintf("[BOOT] decode %s\r\n", ok ? "OK" : "FAILED");

    return boot_cfg;
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart == &hlpuart1) {
        uart_boot_rx_len = Size;
        uart_boot_rx_done = true;
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart == &hlpuart1) {
        uart_boot_rx_error = true;
    }
}

