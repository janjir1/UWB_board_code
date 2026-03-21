#pragma once

/**
 * @file    messages.h
 * @brief   UWB TWR message definitions, offsets, and decoded data structures.
 *
 * Frame wire layout:
 * @code
 * ┌──────────┬──────────┬────────────┬─────────┬─────────────────────┐
 * │ [0..1]   │ [2]      │ [3..4]     │ [5..6]  │ [7..]               │
 * │ reserved │ type     │ sender     │receiver │ payload             │
 * │ 2 bytes  │ 1 byte   │ 2 bytes    │ 2 bytes │ message-specific    │
 * └──────────┴──────────┴────────────┴─────────┴─────────────────────┘
 * @endcode
 */

#include <stdint.h>
#include "uwb_network.h"
#include "DWM3000_driver.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ── Header offsets ─────────────────────────────────────────────────────── */

/** @brief Byte offset of the message type field in the frame buffer. */
#define MSG_START_LOCATION      0

/** @brief Byte offset of the sender ID field (2 bytes, little-endian). */
#define MSG_OFFSET_SENDER       (MSG_START_LOCATION + 1)

/** @brief Byte offset of the receiver ID field (2 bytes, little-endian). */
#define MSG_OFFSET_RECEIVER     (MSG_START_LOCATION + 3)

/** @brief Total length of the message header in bytes (type + sender + receiver). */
#define MSG_HEADER_LEN          (MSG_OFFSET_RECEIVER + 2)

/* ── Timestamp ──────────────────────────────────────────────────────────── */

/** @brief Length of a DW3000 40-bit timestamp as stored on the wire (bytes). */
#define MSG_TS_LEN              5

/* ── Payload offsets ────────────────────────────────────────────────────── */

/** @brief Byte offset of the sequence number, first byte after the header. */
#define MSG_PAYLOAD_OFFSET_SEQ          (MSG_HEADER_LEN)

/** @brief Byte offset of the first timestamp field. */
#define MSG_PAYLOAD_OFFSET_TS_0         (MSG_HEADER_LEN + 1)

/** @brief Byte offset of the second timestamp field. */
#define MSG_PAYLOAD_OFFSET_TS_1         (MSG_HEADER_LEN + 1 + MSG_TS_LEN)

/** @brief Byte offset of the third timestamp field. */
#define MSG_PAYLOAD_OFFSET_TS_2         (MSG_HEADER_LEN + 1 + MSG_TS_LEN * 2)

#define MSG_PAYLOAD_OFFSET_SHARE_SLEEP   (MSG_PAYLOAD_OFFSET_SEQ + 1U)

/**
 * @brief Byte offset of the response RSSI field inside the FINAL message.
 *
 * Carries the RSSI (Q8 fixed-point) measured by the initiator
 * when it received the RESPONSE, transmitted back to the responder.
 */
#define MSG_PAYLOAD_OFFSET_RESP_RSSI    (MSG_HEADER_LEN + 1 + MSG_TS_LEN * 3)

/**
 * @brief Byte offset of the response first-path power field inside the FINAL message.
 *
 * Carries the first-path power (Q8 fixed-point) measured by the initiator
 * when it received the RESPONSE, transmitted back to the responder.
 */
#define MSG_PAYLOAD_OFFSET_RESP_FP      (MSG_HEADER_LEN + 1 + MSG_TS_LEN * 3 + sizeof(int16_t))

/* ── Message type enum ──────────────────────────────────────────────────── */

/**
 * @brief Identifies the role/purpose of a UWB frame.
 *
 * The value is stored at @ref MSG_START_LOCATION in the frame buffer.
 * @c MSG_TYPE_ERR is never sent on the wire — it is used internally
 * to signal a decode failure.
 */
typedef enum {
    MSG_TYPE_ERR      = 0x00, /**< Internal error / decode failure — not transmitted. */
    MSG_TYPE_SYNC     = 0x01, /**< Sync broadcast — announces device presence.       */
    MSG_TYPE_POLL     = 0x02, /**< Poll — initiator starts a TWR exchange.            */
    MSG_TYPE_RESPONSE = 0x03, /**< Response — responder replies to a poll.            */
    MSG_TYPE_FINAL    = 0x04, /**< Final — initiator closes the DS-TWR exchange.      */
    MSG_TYPE_SHARE    = 0x05,
    MSG_TYPE_PASSIVE  = 0x06
} msg_type_t;

/* ── Per-message payload structs ────────────────────────────────────────── */

/**
 * @brief Payload for @c MSG_TYPE_SYNC.
 *
 * Announces presence and carries the list of all currently known peers
 * in the network. Unjoined devices use this list to determine whether
 * they need to request joining.
 */
typedef struct {
    uint8_t  seq_num;                        /**< Sequence number. */
    uint8_t  peer_count;                     /**< Number of peers in the list. */
    uint16_t peer_ids[NETWORK_MAX_PEERS];    /**< IDs of all known active peers, excludes sender. */
} msg_sync_t;

/**
 * @brief Payload for @c MSG_TYPE_POLL.
 *
 * Sent by the initiator to start a ranging exchange.
 * Contains the predicted TX timestamp so the responder can compute
 * turnaround time without waiting for a correction message.
 *
 * @note Fields marked "Not in message" are populated from the
 *       @c dwm_rx_frame_t metadata on the receiver side, not from
 *       the wire payload.
 */
typedef struct {
    uint8_t  seq_num;              /**< Sequence number. */
    /*-- Not in message --*/
    int16_t  poll_rssi_q8;         /**< RSSI of the received POLL (Q8 fixed-point). */
    int16_t  poll_fp_q8;           /**< First-path power of the received POLL (Q8 fixed-point). */
    uint64_t poll_ts;
} msg_poll_t;

/**
 * @brief Payload for @c MSG_TYPE_RESPONSE.
 *
 * Sent by the responder after receiving a POLL.
 * The response carries no timestamps on the wire — the responder's
 * RX/TX timestamps are piggybacked in the FINAL message by the initiator
 * after they are confirmed.
 *
 * @note Fields marked "Not in message" are populated from the
 *       @c dwm_rx_frame_t metadata on the receiver side.
 */
typedef struct {
    uint8_t  seq_num;             /**< Sequence number. */
    /*-- Not in message --*/
    int16_t  response_rssi_q8;    /**< RSSI of the received RESPONSE (Q8 fixed-point). */
    int16_t  response_fp_q8;      /**< First-path power of the received RESPONSE (Q8 fixed-point). */
    uint64_t response_ts;
} msg_response_t;

/**
 * @brief Payload for @c MSG_TYPE_FINAL.
 *
 * Sent by the initiator to close the DS-TWR exchange.
 * Carries all timestamps and signal quality metrics needed by the
 * responder to compute the time-of-flight and derive distance.
 *
 * @note Fields marked "Not in message" are populated from the
 *       @c dwm_rx_frame_t metadata on the receiver side.
 */
typedef struct {
    uint8_t  seq_num;               /**< Sequence number. */
    uint64_t poll_tx_ts;       /**< Actual TX timestamp of the POLL (40-bit, little-endian). */
    uint64_t resp_rx_ts;            /**< RX timestamp of the RESPONSE at the initiator (40-bit). */
    uint64_t final_tx_ts;  /**< Predicted TX timestamp of this FINAL (40-bit). */
    int16_t  resp_rssi_q8;          /**< RSSI measured by initiator on RESPONSE reception (Q8). */
    int16_t  resp_fp_q8;            /**< First-path power on RESPONSE reception at initiator (Q8). */
    /*-- Not in message --*/
    int16_t  final_rssi_q8;         /**< RSSI of the received FINAL at responder (Q8). */
    int16_t  final_fp_q8;           /**< First-path power of the received FINAL at responder (Q8). */
    uint64_t final_ts;
} msg_final_t;

/**
 * @brief Payload for @c MSG_TYPE_SHARE.
 *
 * Broadcast by a node after completing a ranging exchange to share
 * its current sleep/cycle timing with the network, allowing other
 * devices to synchronise their wake schedules.
 *
 * All fields are transmitted on the wire.
 */
typedef struct {
    uint8_t  seq_num;      /**< Sequence number. */
    uint32_t sleep_time;   /**< Planned sleep duration in ms before next cycle. */
} msg_share_t;


typedef struct {
    uint8_t  seq_num;         /**< Sequence number matching the TWR exchange observed. */

    /* --- Passive observations of the TWR exchange (in this node's clock domain) --- */
    uint64_t poll_rx_ts;      /**< RX timestamp of POLL at this passive node (40-bit). */
    uint64_t resp_rx_ts;      /**< RX timestamp of RESPONSE at this passive node (40-bit). */
    uint64_t final_rx_ts;     /**< RX timestamp of FINAL at this passive node (40-bit). */

    /* --- This node's own PASSIVE transmission --- */
    uint64_t passive_tx_ts;   /**< Predicted TX timestamp of this PASSIVE (40-bit). */

    /* --- Preceding PASSIVE TX timestamps, in peer-list order --- */
    uint8_t  entry_count;                              /**< Number of valid entries. */
    uint64_t entries[NETWORK_MAX_PEERS-2];          /**< TX timestamps of preceding PASSIVEs. */
                                                    // The TWR pair is not in it

    /*-- Not in message --*/
    int16_t  poll_rssi_q8;
    int16_t  resp_rssi_q8;
    int16_t  final_rssi_q8;
} msg_passive_t;

/* ── Unified decoded message container ─────────────────────────────────── */

/**
 * @brief Container for a fully decoded UWB message of any type.
 *
 * After a successful call to @c msg_decode(), @c type identifies which
 * union member is valid. Always check @c type before accessing @c data.
 *
 * The union is sized to the largest member (@c msg_final_t), so this
 * struct is safe to allocate on the stack.
 */
typedef struct {
    msg_type_t type;     /**< Decoded message type — determines valid union member. */
    uint16_t   len;      /**< Size in bytes of the active union member.             */
    uint16_t   sender;   /**< Sender device ID decoded from the frame header.       */
    uint16_t   receiver; /**< Receiver device ID decoded from the frame header.     */
    union {
        msg_sync_t     sync;     /**< Valid when @c type == @c MSG_TYPE_SYNC.     */
        msg_poll_t     poll;     /**< Valid when @c type == @c MSG_TYPE_POLL.     */
        msg_response_t response; /**< Valid when @c type == @c MSG_TYPE_RESPONSE. */
        msg_final_t    final;    /**< Valid when @c type == @c MSG_TYPE_FINAL.    */
        msg_share_t    share;
        msg_passive_t  passive;
    } data;
} msg_t;

//void msg_run_tests(void);

void msg_decode(const dwm_rx_frame_t *receive_frame, msg_t *msg_decoded);
dwm_tx_frame_t msg_encode(const msg_t *msg);

#ifdef __cplusplus
}
#endif
