#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define T_SYNC_RX_SET 500 // should be longer then longest sleep
#define T_SYNC_RX_ANSWER 50 //the smaller the better, but stability issues, needs testing

#define SYNC_TO_SYNC 1 //placeholder values
#define POLL_TO_SYNC 1
#define RESPONSE_TO_SYNC 1
#define FINAL_TO_SYNC 1

#define ALL_ID 0xFFFF

#define SYNC_TIMEOUT_MAX 2

typedef enum {
    UWB_SYNC_MASTER,            /* This device is master, sync window complete */
    UWB_SYNC_SLAVE_ACKNOWLEDGED,/* Own ID was seen in master's peer list */
    UWB_SYNC_SLAVE_PENDING,     /* Replied to master but not yet acknowledged */
    UWB_SYNC_SLAVE_REPLY_FAILED,/* Tried to reply but TX failed twice */
    UWB_SYNC_NEW_MASTER,         /* Timed out, promoted self to master */
    UWB_SYNC_TX_FAILED,         /* Master could not transmit SYNC at all */
    UWB_SYNC_MASTER_NO_REPLY,
    UWB_SYNC_NEW_SLAVE,
} uwb_sync_result_t;

uwb_sync_result_t uwb_sync(uint8_t seq_num);