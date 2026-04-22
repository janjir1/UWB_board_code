/**
 * @file messages.h
 * @brief UWB TWR message definitions, offsets, and decoded data structures.
 *
 * Frame wire layout:
 * @code
 * ┌──────────┬─────────┬─────────┬─────────────────────┐
 * │  [0]     │ [1..2]  │ [3..4]  │ [5..]               │
 * │  type    │ sender  │receiver │ payload             │
 * │  1 byte  │ 2 bytes │ 2 bytes │ message-specific    │
 * └──────────┴─────────┴─────────┴─────────────────────┘
 * @endcode
 */

#pragma once

#include <stdint.h>
//#include "uwb_network.h"
#include "DWM3000_driver.h"

/** @brief Maximum number of peers (including self) the network can track. */
#define NETWORK_MAX_PEERS 7

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
#define MSG_TS_LEN  5

/* ── Payload offsets ────────────────────────────────────────────────────── */

/** @brief Byte offset of the sequence number — first byte after the header. */
#define MSG_PAYLOAD_OFFSET_SEQ      (MSG_HEADER_LEN)

/** @brief Byte offset of the first timestamp field in the payload. */
#define MSG_PAYLOAD_OFFSET_TS_0     (MSG_HEADER_LEN + 1)

/** @brief Byte offset of the second timestamp field in the payload. */
#define MSG_PAYLOAD_OFFSET_TS_1     (MSG_HEADER_LEN + 1 + MSG_TS_LEN)

/** @brief Byte offset of the third timestamp field in the payload. */
#define MSG_PAYLOAD_OFFSET_TS_2     (MSG_HEADER_LEN + 1 + MSG_TS_LEN * 2)

/**
 * @brief Byte offset of the sleep-time field inside the SHARE message.
 *
 * Immediately follows the sequence number byte.
 */
#define MSG_PAYLOAD_OFFSET_SHARE_SLEEP  (MSG_PAYLOAD_OFFSET_SEQ + 1U)

/**
 * @brief Byte offset of the response RSSI field inside the FINAL message.
 *
 * Carries the RSSI (Q8 fixed-point) measured by the initiator when it
 * received the RESPONSE, transmitted back so the responder can store it.
 */
#define MSG_PAYLOAD_OFFSET_RESP_RSSI \
    (MSG_HEADER_LEN + 1 + MSG_TS_LEN * 3)

/**
 * @brief Byte offset of the response first-path power field inside the FINAL message.
 *
 * Carries the first-path power (Q8 fixed-point) measured by the initiator
 * when it received the RESPONSE, transmitted back to the responder.
 */
#define MSG_PAYLOAD_OFFSET_RESP_FP \
    (MSG_HEADER_LEN + 1 + MSG_TS_LEN * 3 + sizeof(int16_t))

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
    MSG_TYPE_SYNC     = 0x01, /**< Sync broadcast — announces device presence. */
    MSG_TYPE_POLL     = 0x02, /**< Poll — initiator starts a TWR exchange. */
    MSG_TYPE_RESPONSE = 0x03, /**< Response — responder replies to a POLL. */
    MSG_TYPE_FINAL    = 0x04, /**< Final — initiator closes the DS-TWR exchange. */
    MSG_TYPE_SHARE    = 0x05, /**< Share — broadcasts sleep timing after a ranging round. */
    MSG_TYPE_PASSIVE  = 0x06, /**< Passive — passive observer broadcasts its TWR observations. */
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
    uint8_t  seq_num;                       /**< Sequence number. */
    uint8_t  peer_count;                    /**< Number of peers in the list. */
    uint16_t peer_ids[NETWORK_MAX_PEERS];   /**< IDs of all known active peers, excludes sender. */
} msg_sync_t;

/**
 * @brief Payload for @c MSG_TYPE_POLL.
 *
 * Sent by the initiator to start a ranging exchange.
 * Only the sequence number is transmitted on the wire; the signal quality
 * metrics and RX timestamp are populated from the DWM3000 frame metadata
 * on the receiver side.
 *
 * @note Fields below the "Not in message" marker are not on the wire.
 */
typedef struct {

    uint8_t  seq_num;       /**< Sequence number. */

} msg_poll_t;

/**
 * @brief Payload for @c MSG_TYPE_RESPONSE.
 *
 * Sent by the responder after receiving a POLL.
 * The RESPONSE carries no timestamps on the wire; the responder's RX/TX
 * timestamps are delivered by the initiator inside the FINAL message.
 * Signal quality metrics are populated from the DWM3000 frame metadata.
 *
 * @note Fields below the "Not in message" marker are not on the wire.
 */
typedef struct {

    uint8_t  seq_num;           /**< Sequence number. */

} msg_response_t;


typedef struct {
    uint8_t  seq_num;           /**< Sequence number. */
    uint64_t poll_tx_ts;        /**< Actual TX timestamp of the POLL (40-bit, little-endian). */
    uint64_t resp_rx_ts;        /**< Initiator's RX timestamp of the RESPONSE (40-bit). */
    uint64_t final_tx_ts;       /**< Predicted TX timestamp of this FINAL (40-bit). */
    int16_t  resp_pwr_diff_q8;      /**< RSSI measured by initiator on RESPONSE reception (Q8). */

    /* --- Preceding PASSIVE TX timestamps, in peer-list order --- */
    uint8_t  entry_count;                       /**< Number of valid entries in @c entries. */
    uint64_t entries[NETWORK_MAX_PEERS - 2];    /**< TX timestamps of preceding PASSIVE frames.
                                                 *   The TWR pair (initiator + responder) never
                                                 *   send passive frames, hence the -2. */
    int16_t entry_pwr_diff_q8[NETWORK_MAX_PEERS - 2];
    bool    entry_antenna_unreliable[NETWORK_MAX_PEERS - 2]; 
    uint16_t entry_id[NETWORK_MAX_PEERS - 2];

    
    float IMU_vel_horiz;
    float IMU_vel_vert;

} msg_final_t;

#define NETWORK_MAX_PAIRS ((NETWORK_MAX_PEERS * (NETWORK_MAX_PEERS - 1)) / 2) 

/**
 * SHARE broadcast payload.
 *
 * The node_ids[] array defines the canonical ordering A, B, C, D...
 * Pair entries follow the upper-triangle order: (0,1),(0,2),(0,3),(1,2),(1,3),(2,3)
 * i.e. for i in 0..node_count-1, for j in i+1..node_count-1 → pair index = i*(2n-i-1)/2 + (j-i-1)
 * Both sides encode/decode using the same deterministic formula — no IDs needed per pair.
 */
typedef struct {
    uint8_t  seq_num;                        /**< Sequence number.                         */
    uint32_t sleep_time;                     /**< Planned sleep duration in ms.             */
    uint8_t  node_count;                     /**< Number of nodes in node_ids[].            */
    uint16_t node_ids[NETWORK_MAX_PEERS];      /**< Canonical node order: A, B, C, D...      */

    /* Per-node IMU (index matches node_ids[]) */
    uint8_t  vel_vert [NETWORK_MAX_PEERS];     /**< Vertical velocity, offset-binary.         */
    uint8_t  vel_horiz[NETWORK_MAX_PEERS];     /**< Horizontal speed, offset-binary.          */

    /* Per-pair distances (upper-triangle order, no IDs stored) */
    uint16_t distance_mm [NETWORK_MAX_PAIRS];  /**< Distance 0–200 m in ~3.05 mm/LSB. wrong name        */
    uint8_t  accuracy    [NETWORK_MAX_PAIRS];  /**< Certainty score (0=unknown, 255=best).    */
} msg_share_t;


typedef struct {
    uint8_t  seq_num;       /**< Sequence number matching the TWR exchange observed. */

    /* --- Passive observations of the active TWR frames --- */
    uint64_t poll_rx_ts;    /**< RX timestamp of POLL at this passive node (40-bit). */
    int16_t  poll_pwr_diff_q8;   /**< RSSI of the received POLL at this node (Q8). */

    uint64_t resp_rx_ts;    /**< RX timestamp of RESPONSE at this passive node (40-bit). */
    int16_t  resp_pwr_diff_q8;   /**< RSSI of the received RESPONSE at this node (Q8). */

    /* --- This node's own PASSIVE transmission --- */
    uint64_t passive_tx_ts; /**< Predicted TX timestamp of this PASSIVE frame (40-bit). */

    /* --- Preceding PASSIVE TX timestamps, in peer-list order --- */
    uint8_t  entry_count;                       /**< Number of valid entries in @c entries. */
    uint64_t entries[NETWORK_MAX_PEERS - 2];    /**< TX timestamps of preceding PASSIVE frames.
                                                 *   The TWR pair (initiator + responder) never
                                                 *   send passive frames, hence the -2. */
    int16_t entry_pwr_diff_q8[NETWORK_MAX_PEERS - 2]; /**< RSSI of the received PASSIVE frames at this node (Q8). */
    bool    entry_antenna_unreliable[NETWORK_MAX_PEERS - 2]; 
    uint16_t entry_ids[NETWORK_MAX_PEERS - 2];    /**< Network IDs of the preceding PASSIVE frames, in the same order as @c entries. */


    float IMU_vel_horiz;
    float IMU_vel_vert;

} msg_passive_t;

/* ── Unified decoded message container ─────────────────────────────────── */

/**
 * @brief Container for a fully decoded UWB message of any type.
 *
 * After a successful call to @ref msg_decode, @c type identifies which
 * union member is valid.  Always check @c type before accessing @c data.
 *
 * The union is sized to the largest member (@c msg_passive_t), so this
 * struct is safe to allocate on the stack.
 */
typedef struct {
    msg_type_t type;        /**< Decoded message type — determines valid union member. */
    uint16_t   len;         /**< Size in bytes of the active union member. */
    uint16_t   sender;      /**< Sender device ID decoded from the frame header. */
    uint16_t   receiver;    /**< Receiver device ID decoded from the frame header. */
    union {
        msg_sync_t     sync;     /**< Valid when @c type == @c MSG_TYPE_SYNC. */
        msg_poll_t     poll;     /**< Valid when @c type == @c MSG_TYPE_POLL. */
        msg_response_t response; /**< Valid when @c type == @c MSG_TYPE_RESPONSE. */
        msg_final_t    final;    /**< Valid when @c type == @c MSG_TYPE_FINAL. */
        msg_share_t    share;    /**< Valid when @c type == @c MSG_TYPE_SHARE. */
        msg_passive_t  passive;  /**< Valid when @c type == @c MSG_TYPE_PASSIVE. */
    } data;
} msg_t;

/* ── Public API ─────────────────────────────────────────────────────────── */

/**
 * @brief Decode a raw RX frame into a typed message struct.
 *
 * Reads the message type and header fields (sender, receiver) from the
 * frame buffer, then dispatches to the matching type-specific decoder
 * to fill the payload union in @p msg_decoded.
 *
 * On failure (frame too short or unknown type), @c msg_decoded->type is
 * set to @c MSG_TYPE_ERR and @c msg_decoded->len is set to 0.
 *
 * @param[in]  receive_frame  Pointer to the raw RX frame from the DWM3000 driver.
 * @param[out] msg_decoded    Pointer to the container struct to populate.
 */
void msg_decode(const dwm_rx_frame_t *receive_frame, msg_t *msg_decoded);

/**
 * @brief Encode a typed message struct into a transmit frame.
 *
 * Returns a @c dwm_tx_frame_t pointing to an internal static buffer.
 * The buffer remains valid until the next call to @c msg_encode.
 * @c tx_timestamp is left as 0 — the caller fills it after scheduling
 * a delayed TX.
 *
 * @param[in] msg  Pointer to the populated message struct to encode.
 * @return @c dwm_tx_frame_t with @c data pointer and @c len set.
 *         @c len == 0 indicates failure (null pointer or unknown type).
 */
dwm_tx_frame_t msg_encode(const msg_t *msg);

/**
 * @brief Serialise a 40-bit DW3xxx timestamp into a 5-byte little-endian buffer.
 *
 * Only the low 40 bits of @p ts are written; the upper 24 bits are ignored.
 *
 * @param[in]  ts   64-bit timestamp value (only bits 0–39 are used).
 * @param[out] buf  Output buffer of at least @ref MSG_TS_LEN bytes.
 */
void u64_to_ts_buf(uint64_t ts, uint8_t *buf);

#ifdef __cplusplus
}
#endif