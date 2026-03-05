#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "DWM3000_setup.h"

typedef enum {
    DWM_RX_OK,
    DWM_RX_ERR,
    DWM_RX_TIMEOUT,
} dwm_rx_event_type_t;

typedef struct {
    dwm_rx_event_type_t type;
    uint32_t            status;
    uint8_t             data[DWM_MAX_FRAME_LEN];
    uint16_t            len;
    uint8_t             ts_buf[5];
    dwt_cirdiags_t      rx_diag;  

} dwm_raw_frame_t;

typedef struct {
    dwm_rx_event_type_t type;
    uint32_t            status;
    uint8_t             data[DWM_MAX_FRAME_LEN];
    uint16_t            len;
    uint64_t            rx_timestamp;
    int16_t             rssi_q8;
    int16_t             fp_q8; 

} dwm_frame_t;

extern QueueHandle_t rx_queue;


void cb_rx_ok(const dwt_cb_data_t *cb_data);
void cb_rx_err(const dwt_cb_data_t *cb_data);
void cb_rx_to(const dwt_cb_data_t *cb_data);

void dwm_rx_continuous(void);
void dwm_rx(dwm_frame_t *result, uint32_t timeout_ms);
void dwm_tx_test(void);



#ifdef __cplusplus
}
#endif
