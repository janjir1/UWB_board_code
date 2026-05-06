#ifndef UART_BOOT_H
#define UART_BOOT_H

#include <stdbool.h>
#include <stdint.h>
#include "main.h"

#define UART_BOOT_RX_LEN 64

typedef struct {
    uint16_t id;
    float x;
    float y;
    float z;
} ekf_node_hint_t;

typedef struct {
    bool charge_only;
    ekf_node_hint_t hints[4];
    bool valid;
} boot_config_t;

bool uart_boot_start(void);
boot_config_t uart_boot_config_read(void);

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart);
bool uart_periodic_poll(void);
void uart_rst_arm(void);
void uart_print_power(void);

#endif /* UART_BOOT_H */