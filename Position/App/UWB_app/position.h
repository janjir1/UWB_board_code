#pragma once

#include "uwb_exchange.h"   /* uwb_etwr_result_t */
#include "messages.h"

void distance_calculate(uwb_etwr_result_t result);

uint64_t position_calibrate_timestamp(uint64_t orig_timestamp);

/**
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

    float    init_vel_horiz;
    float    init_vel_vert; 
    float    responder_vel_horiz; 
    float    responder_vel_vert; 

    twr_timestamps_t twr;              

} first_order_t;

typedef struct {
    uint16_t initiator_id;
    uint16_t responder_id;

    float    init_vel_horiz;
    float    init_vel_vert; 
    float    responder_vel_horiz; 
    float    responder_vel_vert; 

    twr_observation_t twr_observation; 

    ss_twr_t twr;              

} second_order_t;

typedef struct {
    uint16_t initiator_id;
    uint16_t responder_id;

    float    init_vel_horiz;
    float    init_vel_vert; 
    float    responder_vel_horiz; 
    float    responder_vel_vert; 

    twr_observation_t twr_observation; 

    ss_twr_t twr; 

}  third_order_t;

typedef struct {

    first_order_t  first;                              // always exactly 1

    second_order_t second[2*(NETWORK_MAX_PEERS - 1)];
    uint8_t        second_count;

    third_order_t  third[(NETWORK_MAX_PEERS - 1)*(NETWORK_MAX_PEERS - 2)/2]; 
    uint8_t        third_count;

} timestamps_t;