#include "messages.h"
#include "DWM3000_driver.h"
#include "../Generic/my_print.h"
#include <stdint.h>
#include <string.h>

#include "cmsis_os.h"
#include "cmsis_os2.h"

/* ── Forward declarations ───────────────────────────────────────────────── */
static void msg_decode_sync    (const dwm_rx_frame_t *frame, msg_sync_t     *out);
static void msg_decode_poll    (const dwm_rx_frame_t *frame, msg_poll_t     *out);
static void msg_decode_response(const dwm_rx_frame_t *frame, msg_response_t *out);
static void msg_decode_final   (const dwm_rx_frame_t *frame, msg_final_t    *out);
static void msg_decode_share   (const dwm_rx_frame_t *frame, msg_share_t    *out);
static void msg_decode_passive (const dwm_rx_frame_t *frame, msg_passive_t  *out);

static uint16_t msg_encode_sync    (const msg_sync_t *in, uint8_t *buf);
static uint16_t msg_encode_poll    (const msg_poll_t *in, uint8_t *buf);
static uint16_t msg_encode_response(const msg_response_t *in, uint8_t *buf);
static uint16_t msg_encode_final   (const msg_final_t *in, uint8_t *buf);
static uint16_t msg_encode_share   (const msg_share_t *in, uint8_t *buf);
static uint16_t msg_encode_passive (const msg_passive_t *in, uint8_t *buf);

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
 * The caller should check @c msg_decoded->type before accessing @c data.
 *
 * @param[in]  receive_frame  Pointer to the raw RX frame from the DWM3000 driver.
 * @param[out] msg_decoded    Pointer to the container struct to populate.
 */

 void u64_to_ts_buf(uint64_t ts, uint8_t *buf)
{
    for (int i = 0; i < MSG_TS_LEN; i++)
        buf[i] = (ts >> (i * 8)) & 0xFF;
}

void msg_decode(const dwm_rx_frame_t *receive_frame, msg_t *msg_decoded)
{
    if (receive_frame->len < MSG_PAYLOAD_OFFSET_SEQ) {
        msg_decoded->type = MSG_TYPE_ERR;
        msg_decoded->len  = 0;
        mprintf("Message too short");
        return;
    }

    msg_decoded->type = (msg_type_t)receive_frame->data[MSG_START_LOCATION];

    memcpy(&msg_decoded->sender,   &receive_frame->data[MSG_OFFSET_SENDER],   sizeof(uint16_t));
    memcpy(&msg_decoded->receiver, &receive_frame->data[MSG_OFFSET_RECEIVER], sizeof(uint16_t));

    switch (msg_decoded->type) {
        case MSG_TYPE_SYNC:
            msg_decode_sync    (receive_frame, &msg_decoded->data.sync);
            msg_decoded->len = sizeof(msg_sync_t);
            break;

        case MSG_TYPE_POLL:
            msg_decode_poll    (receive_frame, &msg_decoded->data.poll);
            msg_decoded->len = sizeof(msg_poll_t);
            break;

        case MSG_TYPE_RESPONSE:
            msg_decode_response(receive_frame, &msg_decoded->data.response);
            msg_decoded->len = sizeof(msg_response_t);
            break;

        case MSG_TYPE_FINAL:
            msg_decode_final   (receive_frame, &msg_decoded->data.final);
            msg_decoded->len = sizeof(msg_final_t);
            break;

        case MSG_TYPE_SHARE:
            msg_decode_share   (receive_frame, &msg_decoded->data.share);
            msg_decoded->len = sizeof(msg_share_t);
            break;
        case MSG_TYPE_PASSIVE:
            msg_decode_passive   (receive_frame, &msg_decoded->data.passive);
            msg_decoded->len = sizeof(msg_passive_t);
            break;

        default:
            msg_decoded->type = MSG_TYPE_ERR;
            msg_decoded->len  = 0;
            break;
    }
}

/**
 * @brief Encode a message into a static internal buffer.
 *
 * Returns a dwm_tx_frame_t pointing to an internal static buffer.
 * The buffer is valid until the next call to msg_encode().
 * tx_timestamp is left empty — caller fills it after delayed TX is scheduled.
 *
 * @param[in]  msg   Pointer to the populated message struct to encode.
 * @return           dwm_tx_frame_t with data pointer and length.
 *                   len == 0 indicates failure.
 */
dwm_tx_frame_t msg_encode(const msg_t *msg)
{
    static uint8_t buf[DWM_MAX_FRAME_LEN];

    dwm_tx_frame_t frame = { .data = buf, .len = 0, .tx_timestamp = 0 };

    if (!msg) return frame;

    buf[MSG_START_LOCATION] = (uint8_t)msg->type;
    memcpy(&buf[MSG_OFFSET_SENDER],   &msg->sender,   sizeof(uint16_t));
    memcpy(&buf[MSG_OFFSET_RECEIVER], &msg->receiver, sizeof(uint16_t));

    switch (msg->type) {
        case MSG_TYPE_SYNC:
            frame.len = msg_encode_sync    (&msg->data.sync,     buf);  break;
        case MSG_TYPE_POLL:
            frame.len = msg_encode_poll    (&msg->data.poll,     buf);  break;
        case MSG_TYPE_RESPONSE:
            frame.len = msg_encode_response(&msg->data.response, buf);  break;
        case MSG_TYPE_FINAL:
            frame.len = msg_encode_final   (&msg->data.final,    buf);  break;
        case MSG_TYPE_SHARE:
            frame.len = msg_encode_share   (&msg->data.share,    buf);  break;
        case MSG_TYPE_PASSIVE:
            frame.len = msg_encode_passive (&msg->data.passive,    buf);  break;
        default:
            break;
    }

    return frame;
}

/* ── Private decoders/encoders ───────────────────────────────────────────────────── */

/**
 * @brief Decode a SYNC message payload.
 *
 * @param[in]  frame  Raw RX frame.
 * @param[out] out    Decoded sync struct to populate.
 */
static void msg_decode_sync(const dwm_rx_frame_t *frame, msg_sync_t *out)
{
    out->seq_num    = frame->data[MSG_PAYLOAD_OFFSET_SEQ];
    out->peer_count = frame->data[MSG_PAYLOAD_OFFSET_SEQ + 1];

    for (int i = 0; i < out->peer_count && i < NETWORK_MAX_PEERS; i++)
        memcpy(&out->peer_ids[i],
               &frame->data[MSG_PAYLOAD_OFFSET_SEQ + 2 + i * sizeof(uint16_t)],
               sizeof(uint16_t));
}

/**
 * @brief Encode a SYNC message payload into the wire buffer.
 *
 * Wire layout:
 * @code
 * [SEQ]        1 byte
 * [PEER_COUNT] 1 byte
 * [PEER_IDS]   peer_count × 2 bytes
 * @endcode
 *
 * @param[in]  in   Populated sync struct.
 * @param[out] buf  Output buffer.
 * @return          Total bytes written.
 */
static uint16_t msg_encode_sync(const msg_sync_t *in, uint8_t *buf)
{
    buf[MSG_PAYLOAD_OFFSET_SEQ]          = in->seq_num;
    buf[MSG_PAYLOAD_OFFSET_SEQ + 1]      = in->peer_count;

    for (int i = 0; i < in->peer_count; i++)
        memcpy(&buf[MSG_PAYLOAD_OFFSET_SEQ + 2 + i * sizeof(uint16_t)],
               &in->peer_ids[i], sizeof(uint16_t));

    return MSG_PAYLOAD_OFFSET_SEQ + 2 + in->peer_count * sizeof(uint16_t);
}

/**
 * @brief Decode a POLL message payload.
 *
 * Extracts the sequence number.
 * RSSI and first-path power are taken from the frame metadata, not the
 * wire payload.
 *
 * @param[in]  frame  Raw RX frame.
 * @param[out] out    Decoded poll struct to populate.
 */
static void msg_decode_poll(const dwm_rx_frame_t *frame, msg_poll_t *out)
{
    out->seq_num             = frame->data[MSG_PAYLOAD_OFFSET_SEQ];
    /* -- Not in message: populated from RX frame metadata -- */
    out->poll_rssi_q8        = frame->rssi_q8;
    out->poll_fp_q8          = frame->fp_q8;
    out->poll_ts             = frame->rx_timestamp;
}

/**
 * @brief Encode a POLL message payload into the wire buffer.
 *
 * Wire layout:
 * @code
 * [SEQ] 1 byte
 * @endcode
 *
 * @note poll_rssi_q8 and poll_fp_q8 are not transmitted —
 *       they are measured locally by the receiver.
 *
 * @param[in]  in   Populated poll struct.
 * @param[out] buf  Output buffer, must be at least MSG_PAYLOAD_OFFSET_TS_0 + MSG_TS_LEN bytes.
 * @return          Total bytes written.
 */
static uint16_t msg_encode_poll(const msg_poll_t *in, uint8_t *buf)
{
    buf[MSG_PAYLOAD_OFFSET_SEQ] = in->seq_num;
    return MSG_PAYLOAD_OFFSET_SEQ + 1;
}

/**
 * @brief Decode a RESPONSE message payload.
 *
 * The RESPONSE carries no timestamps on the wire. RSSI and first-path
 * power are taken from the frame metadata.
 *
 * @param[in]  frame  Raw RX frame.
 * @param[out] out    Decoded response struct to populate.
 */
static void msg_decode_response(const dwm_rx_frame_t *frame, msg_response_t *out)
{
    out->seq_num          = frame->data[MSG_PAYLOAD_OFFSET_SEQ];
    /* -- Not in message: populated from RX frame metadata -- */
    out->response_rssi_q8 = frame->rssi_q8;
    out->response_fp_q8   = frame->fp_q8;
    out->response_ts             = frame->rx_timestamp;
}

/**
 * @brief Encode a RESPONSE message payload into the wire buffer.
 *
 * Wire layout:
 * @code
 * [SEQ] 1 byte
 * @endcode
 *
 * @note The RESPONSE carries no timestamps on the wire. The responder's
 *       RX/TX timing data is delivered by the initiator inside the FINAL
 *       message. response_rssi_q8 and response_fp_q8 are not transmitted —
 *       they are measured locally by the receiver.
 *
 * @param[in]  in   Populated response struct.
 * @param[out] buf  Output buffer, must be at least MSG_PAYLOAD_OFFSET_SEQ + 1 bytes.
 * @return          Total bytes written.
 */
static uint16_t msg_encode_response(const msg_response_t *in, uint8_t *buf)
{
    buf[MSG_PAYLOAD_OFFSET_SEQ] = in->seq_num;
    return MSG_PAYLOAD_OFFSET_SEQ + 1;
}

/**
 * @brief Decode a FINAL message payload.
 *
 * Extracts all three initiator timestamps and the RSSI/FP values the
 * initiator measured when it received the RESPONSE. Those values were
 * transmitted by the initiator so the responder has a complete picture
 * of link quality across all three messages:
 *  - POLL   reception quality: stored by responder when POLL arrived.
 *  - RESPONSE reception quality: decoded here from the wire payload.
 *  - FINAL  reception quality: read from current frame metadata.
 *
 * @param[in]  frame  Raw RX frame.
 * @param[out] out    Decoded final struct to populate.
 */
static void msg_decode_final(const dwm_rx_frame_t *frame, msg_final_t *out)
{
    out->seq_num              = frame->data[MSG_PAYLOAD_OFFSET_SEQ];
    out->poll_tx_ts           = ts_buf_to_u64(&frame->data[MSG_PAYLOAD_OFFSET_TS_0]);
    out->resp_rx_ts           = ts_buf_to_u64(&frame->data[MSG_PAYLOAD_OFFSET_TS_1]);
    out->final_tx_ts = ts_buf_to_u64(&frame->data[MSG_PAYLOAD_OFFSET_TS_2]);
    /* -- Sent by initiator: quality of RESPONSE as seen by initiator -- */
    memcpy(&out->resp_rssi_q8, &frame->data[MSG_PAYLOAD_OFFSET_RESP_RSSI], sizeof(int16_t));
    memcpy(&out->resp_fp_q8,   &frame->data[MSG_PAYLOAD_OFFSET_RESP_FP],   sizeof(int16_t));
    /* -- Not in message: quality of FINAL as seen by responder -- */
    out->final_rssi_q8        = frame->rssi_q8;
    out->final_fp_q8          = frame->fp_q8;
    out->final_ts             = frame->rx_timestamp;
}

/**
 * @brief Encode a FINAL message payload into the wire buffer.
 *
 * Wire layout:
 * @code
 * [SEQ]        1 byte
 * [TS_0]       5 bytes  poll_tx_real_ts      — actual TX time of the POLL
 * [TS_1]       5 bytes  resp_rx_ts           — initiator RX time of the RESPONSE
 * [TS_2]       5 bytes  final_tx_expected_ts — predicted TX time of this FINAL
 * [RESP_RSSI]  2 bytes  resp_rssi_q8         — RSSI at initiator on RESPONSE (Q8)
 * [RESP_FP]    2 bytes  resp_fp_q8           — first-path power at initiator on RESPONSE (Q8)
 * @endcode
 *
 * @note final_rssi_q8 and final_fp_q8 are not transmitted — they are
 *       measured locally by the responder when this FINAL arrives.
 *
 * @param[in]  in   Populated final struct.
 * @param[out] buf  Output buffer, must be at least MSG_PAYLOAD_OFFSET_RESP_FP + sizeof(int16_t) bytes.
 * @return          Total bytes written.
 */
static uint16_t msg_encode_final(const msg_final_t *in, uint8_t *buf)
{
    buf[MSG_PAYLOAD_OFFSET_SEQ] = in->seq_num;
    u64_to_ts_buf(in->poll_tx_ts,      &buf[MSG_PAYLOAD_OFFSET_TS_0]);
    u64_to_ts_buf(in->resp_rx_ts,           &buf[MSG_PAYLOAD_OFFSET_TS_1]);
    u64_to_ts_buf(in->final_tx_ts, &buf[MSG_PAYLOAD_OFFSET_TS_2]);
    memcpy(&buf[MSG_PAYLOAD_OFFSET_RESP_RSSI], &in->resp_rssi_q8, sizeof(int16_t));
    memcpy(&buf[MSG_PAYLOAD_OFFSET_RESP_FP],   &in->resp_fp_q8,   sizeof(int16_t));
    return MSG_PAYLOAD_OFFSET_RESP_FP + sizeof(int16_t);
}

/**
 * @brief Decode a SHARE message payload.
 *
 * Extracts the sender's planned sleep time so the receiver can align
 * its own wake schedule to the sender's next cycle.
 *
 * @param[in]  frame  Raw RX frame.
 * @param[out] out    Decoded share struct to populate.
 */
static void msg_decode_share(const dwm_rx_frame_t *frame, msg_share_t *out)
{
    out->seq_num = frame->data[MSG_PAYLOAD_OFFSET_SEQ];
    memcpy(&out->sleep_time, &frame->data[MSG_PAYLOAD_OFFSET_SHARE_SLEEP],
           sizeof(uint32_t));
}

/**
 * @brief Encode a SHARE message payload into the wire buffer.
 *
 * Wire layout:
 * @code
 * [SEQ]        1 byte
 * [SLEEP]      4 bytes  sleep_time — ms until next cycle start
 * @endcode
 *
 * @param[in]  in   Populated share struct.
 * @param[out] buf  Output buffer, must be at least
 *                  @c MSG_PAYLOAD_OFFSET_SHARE_SLEEP + sizeof(uint32_t) bytes.
 * @return          Total bytes written.
 */
static uint16_t msg_encode_share(const msg_share_t *in, uint8_t *buf)
{
    buf[MSG_PAYLOAD_OFFSET_SEQ] = in->seq_num;
    memcpy(&buf[MSG_PAYLOAD_OFFSET_SHARE_SLEEP], &in->sleep_time,
           sizeof(uint32_t));
    return MSG_PAYLOAD_OFFSET_SHARE_SLEEP + sizeof(uint32_t);
}


/**
 * @brief Decode a PASSIVE message payload.
 *
 * @param[in]  frame  Raw RX frame.
 * @param[out] out    Decoded passive struct to populate.
 */
static void msg_decode_passive(const dwm_rx_frame_t *frame, msg_passive_t *out)
{
    const uint8_t *p = &frame->data[MSG_PAYLOAD_OFFSET_SEQ];

    /* Zero upper bytes of all uint64_t fields — only 5 bytes come off the wire. */
    memset(out, 0, sizeof(*out));

    out->seq_num = *p++;

    memcpy(&out->poll_rx_ts,    p, MSG_TS_LEN); p += MSG_TS_LEN;
    memcpy(&out->resp_rx_ts,    p, MSG_TS_LEN); p += MSG_TS_LEN;
    memcpy(&out->final_rx_ts,   p, MSG_TS_LEN); p += MSG_TS_LEN;
    memcpy(&out->passive_tx_ts, p, MSG_TS_LEN); p += MSG_TS_LEN;

    out->entry_count = *p++;

    for (int i = 0; i < out->entry_count && i < (NETWORK_MAX_PEERS - 2); i++) {
        memcpy(&out->entries[i], p, MSG_TS_LEN);
        p += MSG_TS_LEN;
    }
}

/**
 * @brief Encode a PASSIVE message payload into the wire buffer.
 *
 * Wire layout:
 * @code
 * [SEQ]           1 byte
 * [POLL_RX_TS]    5 bytes  (40-bit, little-endian)
 * [RESP_RX_TS]    5 bytes
 * [FINAL_RX_TS]   5 bytes
 * [PASSIVE_TX_TS] 5 bytes
 * [ENTRY_COUNT]   1 byte
 * [ENTRIES]       entry_count × 5 bytes
 * @endcode
 *
 * @param[in]  in   Populated passive struct.
 * @param[out] buf  Output buffer.
 * @return          Total bytes written.
 */
static uint16_t msg_encode_passive(const msg_passive_t *in, uint8_t *buf)
{
    uint8_t *p = &buf[MSG_PAYLOAD_OFFSET_SEQ];

    *p++ = in->seq_num;

    memcpy(p, &in->poll_rx_ts,    MSG_TS_LEN); p += MSG_TS_LEN;
    memcpy(p, &in->resp_rx_ts,    MSG_TS_LEN); p += MSG_TS_LEN;
    memcpy(p, &in->final_rx_ts,   MSG_TS_LEN); p += MSG_TS_LEN;
    memcpy(p, &in->passive_tx_ts, MSG_TS_LEN); p += MSG_TS_LEN;

    *p++ = in->entry_count;

    for (int i = 0; i < in->entry_count; i++) {
        memcpy(p, &in->entries[i], MSG_TS_LEN);
        p += MSG_TS_LEN;
    }

    return (uint16_t)(p - buf);
}

static dwm_rx_frame_t make_rx_frame(const dwm_tx_frame_t *tx, int16_t rssi, int16_t fp)
{
    dwm_rx_frame_t rx;
    memset(&rx, 0, sizeof(rx));
    memcpy(rx.data, tx->data, tx->len);
    rx.len     = tx->len;
    rx.rssi_q8 = rssi;
    rx.fp_q8   = fp;
    return rx;
}

#ifdef UWB_DEBUG
//This function may not work properly due to changes in messages.h, function was only fixed so it compiles
//Helper macro to print a 40-bit timestamp as two 32-bit halves 
#define MPRINT_TS(label, ts) \
    mprintf(label "0x%08lX%08lX", (uint32_t)((ts) >> 32), (uint32_t)((ts) & 0xFFFFFFFF))

static void test_sync(void)
{
    msg_t tx_msg = {
        .type      = MSG_TYPE_SYNC,
        .sender    = 0x0011,
        .receiver  = 0x0022,
        .data.sync = { .seq_num = 0xAB }
    };

    dwm_tx_frame_t tx_frame = msg_encode(&tx_msg);
    dwm_rx_frame_t rx_frame = make_rx_frame(&tx_frame, 0, 0);
    msg_t rx_msg;
    msg_decode(&rx_frame, &rx_msg);

    mprintf("SYNC  IN:  seq=0x%02X\r\n",    tx_msg.data.sync.seq_num);
    mprintf("SYNC  OUT: seq=0x%02X\r\n\r\n", rx_msg.data.sync.seq_num);
}

static void test_poll(void)
{
    msg_t tx_msg = {
        .type      = MSG_TYPE_POLL,
        .sender    = 0x0033,
        .receiver  = 0x0044,
        .data.poll = { .seq_num = 0x05}
    };

    dwm_tx_frame_t tx_frame = msg_encode(&tx_msg);
    dwm_rx_frame_t rx_frame = make_rx_frame(&tx_frame, -512, -256);
    msg_t rx_msg;
    msg_decode(&rx_frame, &rx_msg);

    mprintf("POLL  IN:  seq=0x%02X\r\n", tx_msg.data.poll.seq_num);  mprintf("\r\n");
    mprintf("POLL  OUT: seq=0x%02X  ts=", rx_msg.data.poll.seq_num);
    MPRINT_TS("", rx_msg.data.poll.poll_ts);
    mprintf("  rssi=%d  fp=%d\r\n\r\n", rx_msg.data.poll.poll_rssi_q8, rx_msg.data.poll.poll_fp_q8);
}

static void test_response(void)
{
    msg_t tx_msg = {
        .type          = MSG_TYPE_RESPONSE,
        .sender        = 0x0055,
        .receiver      = 0x0066,
        .data.response = { .seq_num = 0x07 }
    };

    dwm_tx_frame_t tx_frame = msg_encode(&tx_msg);
    dwm_rx_frame_t rx_frame = make_rx_frame(&tx_frame, -300, -150);
    msg_t rx_msg;
    msg_decode(&rx_frame, &rx_msg);

    mprintf("RESP  IN:  seq=0x%02X\r\n", tx_msg.data.response.seq_num);
    mprintf("RESP  OUT: seq=0x%02X  rssi=%d  fp=%d\r\n\r\n",
            rx_msg.data.response.seq_num,
            rx_msg.data.response.response_rssi_q8,
            rx_msg.data.response.response_fp_q8);
}

static void test_final(void)
{
    msg_t tx_msg = {
        .type       = MSG_TYPE_FINAL,
        .sender     = 0x0077,
        .receiver   = 0x0088,
        .data.final = {
            .seq_num              = 0x09,
            .poll_tx_ts      = 0x0000001111111111ULL,
            .resp_rx_ts           = 0x0000002222222222ULL,
            .final_tx_ts = 0x0000003333333333ULL,
            .resp_rssi_q8         = -400,
            .resp_fp_q8           = -200,
        }
    };

    dwm_tx_frame_t tx_frame = msg_encode(&tx_msg);
    dwm_rx_frame_t rx_frame = make_rx_frame(&tx_frame, -600, -100);
    msg_t rx_msg;
    msg_decode(&rx_frame, &rx_msg);

    mprintf("FINAL IN:  seq=0x%02X\r\n", tx_msg.data.final.seq_num);
    MPRINT_TS("  poll_tx=",  tx_msg.data.final.poll_tx_ts);      mprintf("\r\n");
    MPRINT_TS("  resp_rx=",  tx_msg.data.final.resp_rx_ts);           mprintf("\r\n");
    MPRINT_TS("  final_tx=", tx_msg.data.final.final_tx_ts); mprintf("\r\n");
    mprintf("  resp_rssi=%d  resp_fp=%d\r\n", tx_msg.data.final.resp_rssi_q8, tx_msg.data.final.resp_fp_q8);

    mprintf("FINAL OUT: seq=0x%02X\r\n", rx_msg.data.final.seq_num);
    MPRINT_TS("  poll_tx=",  rx_msg.data.final.poll_tx_ts);      mprintf("\r\n");
    MPRINT_TS("  resp_rx=",  rx_msg.data.final.resp_rx_ts);           mprintf("\r\n");
    MPRINT_TS("  final_tx=", rx_msg.data.final.final_tx_ts); mprintf("\r\n");
    mprintf("  resp_rssi=%d  resp_fp=%d\r\n",   rx_msg.data.final.resp_rssi_q8,  rx_msg.data.final.resp_fp_q8);
    mprintf("  final_rssi=%d  final_fp=%d\r\n\r\n", rx_msg.data.final.final_rssi_q8, rx_msg.data.final.final_fp_q8);
}


//@brief Run all message encode/decode roundtrip tests.
 
void msg_run_tests(void)
{
    mprintf("=== msg roundtrip tests ===\r\n\r\n");
    test_sync();
    test_poll();
    test_response();
    test_final();
    mprintf("=== tests done ===\r\n");
}

#endif