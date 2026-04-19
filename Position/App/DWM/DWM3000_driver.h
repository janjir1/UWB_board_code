#pragma once

#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif

#include "DWM3000_setup.h"

#define US_TO_DWT_TIME   65536U
#define MS_TO_DWT_TIME (US_TO_DWT_TIME * 1000ULL)

#define TX_DELAY_500US    (5000U * US_TO_DWT_TIME)

typedef enum {
    DWM_RX_OK,
    DWM_RX_ERR,
    DWM_RX_TIMEOUT,
} dwm_rx_event_type_t;

typedef enum {
    DWM_TX_OK,
    DWM_TX_LATE,
    DWM_TX_ERROR,
} dwm_tx_event_type_t;


typedef struct {
    dwm_rx_event_type_t type;
    uint32_t            status;
    uint8_t             data[DWM_MAX_FRAME_LEN];
    uint16_t            len;
    uint8_t             ts_buf[5];
    dwt_cirdiags_t      rx_diag;  

} dwm_rx_raw_frame_t;

typedef struct {
    dwm_rx_event_type_t type;
    uint32_t            status;
    uint8_t             data[DWM_MAX_FRAME_LEN];
    uint16_t            len;
    uint64_t            rx_timestamp;
    int16_t             rssi_q8;
    int16_t             fp_q8; 

} dwm_rx_frame_t;

typedef struct {
    uint8_t  *data;
    uint16_t  len;
    uint64_t  tx_timestamp;
} dwm_tx_frame_t;

uint64_t ts_buf_to_u64(const uint8_t buf[5]);

bool dwm_init(void);
bool dwm_selftest(void);
bool dwm_configure(void);
uint16_t dwm_get_addr(void);


void cb_rx_ok(const dwt_cb_data_t *cb_data);
void cb_rx_err(const dwt_cb_data_t *cb_data);
void cb_rx_to(const dwt_cb_data_t *cb_data);
void cb_tx_done(const dwt_cb_data_t *cb_data);
void cb_spi_rdy(const dwt_cb_data_t *cb_data);

void dwm_rx(dwm_rx_frame_t *result, uint32_t timeout_ms, bool keep_listening, bool first_call);
void dwm_rx_flush(void);

dwm_tx_event_type_t dwm_tx(dwm_tx_frame_t *frame);
dwm_tx_event_type_t dwm_tx_delayed(dwm_tx_frame_t *frame, uint64_t final_rmarker_ts, uint32_t wait_ms);
void dwm_tx_continuous(void);
void dwm_tx_continuous_delayed(void);
void dwm_rx_continuous_sleep(void);

void dwm_wakeup(void);
void dwm_sleep(void);
int dwm_exit_idle_rc(void);
int dwm_enter_idle_rc(void);
#ifdef __cplusplus
}
#endif
