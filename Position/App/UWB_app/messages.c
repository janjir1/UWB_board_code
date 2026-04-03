/**
 * @file messages.c
 * @brief UWB TWR message encode/decode implementation.
 *
 * Implements the public @ref msg_encode / @ref msg_decode API and all
 * private per-type encoder and decoder helpers.
 *
 * Each message type has a matching pair:
 * - @c msg_decode_<type>  — extracts fields from the raw byte buffer.
 * - @c msg_encode_<type>  — writes fields into the raw byte buffer.
 *
 * The @c #ifdef UWB_DEBUG block at the bottom contains round-trip
 * self-tests for every message type; enable by defining @c UWB_DEBUG.
 */

#include "messages.h"
#include "DWM3000_driver.h"
#include "../Generic/my_print.h"
#include <string.h>
#include <stdint.h>

#include "cmsis_os.h"
#include "cmsis_os2.h"

/* ── Forward declarations ───────────────────────────────────────────────── */
static void     msg_decode_sync    (const dwm_rx_frame_t *frame, msg_sync_t     *out);
static void     msg_decode_poll    (const dwm_rx_frame_t *frame, msg_poll_t     *out);
static void     msg_decode_response(const dwm_rx_frame_t *frame, msg_response_t *out);
static void     msg_decode_final   (const dwm_rx_frame_t *frame, msg_final_t    *out);
static void     msg_decode_share   (const dwm_rx_frame_t *frame, msg_share_t    *out);
static void     msg_decode_passive (const dwm_rx_frame_t *frame, msg_passive_t  *out);

static uint16_t msg_encode_sync    (const msg_sync_t     *in, uint8_t *buf);
static uint16_t msg_encode_poll    (const msg_poll_t     *in, uint8_t *buf);
static uint16_t msg_encode_response(const msg_response_t *in, uint8_t *buf);
static uint16_t msg_encode_final   (const msg_final_t    *in, uint8_t *buf);
static uint16_t msg_encode_share   (const msg_share_t    *in, uint8_t *buf);
static uint16_t msg_encode_passive (const msg_passive_t  *in, uint8_t *buf);

/* ── Utility ────────────────────────────────────────────────────────────── */

void u64_to_ts_buf(uint64_t ts, uint8_t *buf)
{
    for (int i = 0; i < MSG_TS_LEN; i++)
        buf[i] = (ts >> (i * 8)) & 0xFF;
}

/* ── Public API ─────────────────────────────────────────────────────────── */

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
        msg_decode_passive (receive_frame, &msg_decoded->data.passive);
        msg_decoded->len = sizeof(msg_passive_t);
        break;
    default:
        msg_decoded->type = MSG_TYPE_ERR;
        msg_decoded->len  = 0;
        break;
    }
}

dwm_tx_frame_t msg_encode(const msg_t *msg)
{
    static uint8_t buf[DWM_MAX_FRAME_LEN];

    dwm_tx_frame_t frame = { .data = buf, .len = 0, .tx_timestamp = 0 };

    if (!msg) return frame;

    buf[MSG_START_LOCATION] = (uint8_t)msg->type;
    memcpy(&buf[MSG_OFFSET_SENDER],   &msg->sender,   sizeof(uint16_t));
    memcpy(&buf[MSG_OFFSET_RECEIVER], &msg->receiver, sizeof(uint16_t));

    switch (msg->type) {
    case MSG_TYPE_SYNC:     frame.len = msg_encode_sync    (&msg->data.sync,     buf); break;
    case MSG_TYPE_POLL:     frame.len = msg_encode_poll    (&msg->data.poll,     buf); break;
    case MSG_TYPE_RESPONSE: frame.len = msg_encode_response(&msg->data.response, buf); break;
    case MSG_TYPE_FINAL:    frame.len = msg_encode_final   (&msg->data.final,    buf); break;
    case MSG_TYPE_SHARE:    frame.len = msg_encode_share   (&msg->data.share,    buf); break;
    case MSG_TYPE_PASSIVE:  frame.len = msg_encode_passive (&msg->data.passive,  buf); break;
    default: break;
    }

    return frame;
}

/* ── Private decoders ───────────────────────────────────────────────────── */

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
 * @brief Decode a POLL message payload.
 *
 * Extracts the sequence number from the wire. RSSI, first-path power,
 * and the RX timestamp are taken from the DWM3000 frame metadata.
 *
 * @param[in]  frame  Raw RX frame.
 * @param[out] out    Decoded poll struct to populate.
 */
static void msg_decode_poll(const dwm_rx_frame_t *frame, msg_poll_t *out)
{
    out->seq_num     = frame->data[MSG_PAYLOAD_OFFSET_SEQ];

}

/**
 * @brief Decode a RESPONSE message payload.
 *
 * The RESPONSE carries no timestamps on the wire. RSSI and first-path
 * power are taken from the DWM3000 frame metadata.
 *
 * @param[in]  frame  Raw RX frame.
 * @param[out] out    Decoded response struct to populate.
 */
static void msg_decode_response(const dwm_rx_frame_t *frame, msg_response_t *out)
{
    out->seq_num          = frame->data[MSG_PAYLOAD_OFFSET_SEQ];
    /* -- Not in message: populated from RX frame metadata -- */

}


static void msg_decode_final(const dwm_rx_frame_t *frame, msg_final_t *out)
{
    const uint8_t *p = &frame->data[MSG_PAYLOAD_OFFSET_SEQ];

    memset(out, 0, sizeof(*out));

    out->seq_num = *p++;

    memcpy(&out->poll_tx_ts,   p, MSG_TS_LEN);      p += MSG_TS_LEN;
    memcpy(&out->resp_rx_ts,   p, MSG_TS_LEN);      p += MSG_TS_LEN;
    memcpy(&out->final_tx_ts,  p, MSG_TS_LEN);      p += MSG_TS_LEN;

    memcpy(&out->resp_rssi_q8, p, sizeof(int16_t)); p += sizeof(int16_t);

    out->entry_count = *p++;
    for (int i = 0; i < out->entry_count && i < (NETWORK_MAX_PEERS - 2); i++) {
        memcpy(&out->entries[i],      p, MSG_TS_LEN);        p += MSG_TS_LEN;
        memcpy(&out->entry_rssi_q8[i], p, sizeof(int16_t)); p += sizeof(int16_t);
        memcpy(&out->entry_id[i],      p, sizeof(uint16_t)); p += sizeof(uint16_t); // ← new
    }

    memcpy(&out->IMU_pitch_rad, p, sizeof(float)); p += sizeof(float);
    memcpy(&out->IMU_vel_horiz, p, sizeof(float)); p += sizeof(float);
    memcpy(&out->IMU_vel_vert,  p, sizeof(float)); p += sizeof(float);
}

/**
 * @brief Decode a SHARE message payload.
 *
 * Extracts the sender's planned sleep time so the receiver can align
 * its own wake schedule to the next cycle.
 *
 * @param[in]  frame  Raw RX frame.
 * @param[out] out    Decoded share struct to populate.
 */
static void msg_decode_share(const dwm_rx_frame_t *frame, msg_share_t *out)
{
    out->seq_num = frame->data[MSG_PAYLOAD_OFFSET_SEQ];
    memcpy(&out->sleep_time,
           &frame->data[MSG_PAYLOAD_OFFSET_SHARE_SLEEP],
           sizeof(uint32_t));
}


static void msg_decode_passive(const dwm_rx_frame_t *frame, msg_passive_t *out)
{
    const uint8_t *p = &frame->data[MSG_PAYLOAD_OFFSET_SEQ];

    memset(out, 0, sizeof(*out));

    out->seq_num = *p++;

    memcpy(&out->poll_rx_ts,   p, MSG_TS_LEN);       p += MSG_TS_LEN;
    memcpy(&out->poll_rssi_q8, p, sizeof(int16_t));  p += sizeof(int16_t);

    memcpy(&out->resp_rx_ts,   p, MSG_TS_LEN);       p += MSG_TS_LEN;
    memcpy(&out->resp_rssi_q8, p, sizeof(int16_t));  p += sizeof(int16_t);

    /* final_rx_ts removed — PASSIVE now transmits before FINAL */

    memcpy(&out->passive_tx_ts, p, MSG_TS_LEN); p += MSG_TS_LEN;

    out->entry_count = *p++;
    for (int i = 0; i < out->entry_count && i < (NETWORK_MAX_PEERS - 2); i++) {
        memcpy(&out->entries[i],       p, MSG_TS_LEN);       p += MSG_TS_LEN;
        memcpy(&out->entry_rssi_q8[i], p, sizeof(int16_t)); p += sizeof(int16_t);
        memcpy(&out->entry_ids[i],     p, sizeof(uint16_t)); p += sizeof(uint16_t);
    }

    memcpy(&out->IMU_pitch_rad, p, sizeof(float)); p += sizeof(float);
    memcpy(&out->IMU_vel_horiz, p, sizeof(float)); p += sizeof(float);
    memcpy(&out->IMU_vel_vert,  p, sizeof(float)); p += sizeof(float);
}

/* ── Private encoders ───────────────────────────────────────────────────── */

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
 * @param[out] buf  Output buffer (at least MSG_PAYLOAD_OFFSET_SEQ + 2 +
 *                  peer_count × 2 bytes).
 * @return Total bytes written.
 */
static uint16_t msg_encode_sync(const msg_sync_t *in, uint8_t *buf)
{
    buf[MSG_PAYLOAD_OFFSET_SEQ]     = in->seq_num;
    buf[MSG_PAYLOAD_OFFSET_SEQ + 1] = in->peer_count;

    for (int i = 0; i < in->peer_count; i++)
        memcpy(&buf[MSG_PAYLOAD_OFFSET_SEQ + 2 + i * sizeof(uint16_t)],
               &in->peer_ids[i], sizeof(uint16_t));

    return MSG_PAYLOAD_OFFSET_SEQ + 2 + in->peer_count * sizeof(uint16_t);
}

/**
 * @brief Encode a POLL message payload into the wire buffer.
 *
 * Wire layout:
 * @code
 * [SEQ]  1 byte
 * @endcode
 *
 * @note @c poll_rssi_q8 and @c poll_fp_q8 are not transmitted —
 *       they are measured locally by the receiver.
 *
 * @param[in]  in   Populated poll struct.
 * @param[out] buf  Output buffer (at least MSG_PAYLOAD_OFFSET_SEQ + 1 bytes).
 * @return Total bytes written.
 */
static uint16_t msg_encode_poll(const msg_poll_t *in, uint8_t *buf)
{
    buf[MSG_PAYLOAD_OFFSET_SEQ] = in->seq_num;
    return MSG_PAYLOAD_OFFSET_SEQ + 1;
}

/**
 * @brief Encode a RESPONSE message payload into the wire buffer.
 *
 * Wire layout:
 * @code
 * [SEQ]  1 byte
 * @endcode
 *
 * @note The RESPONSE carries no timestamps on the wire. The responder's
 *       RX/TX timing data is delivered by the initiator inside the FINAL.
 *       @c response_rssi_q8 and @c response_fp_q8 are not transmitted.
 *
 * @param[in]  in   Populated response struct.
 * @param[out] buf  Output buffer (at least MSG_PAYLOAD_OFFSET_SEQ + 1 bytes).
 * @return Total bytes written.
 */
static uint16_t msg_encode_response(const msg_response_t *in, uint8_t *buf)
{
    buf[MSG_PAYLOAD_OFFSET_SEQ] = in->seq_num;
    return MSG_PAYLOAD_OFFSET_SEQ + 1;
}


static uint16_t msg_encode_final(const msg_final_t *in, uint8_t *buf)
{
    uint8_t *p = &buf[MSG_PAYLOAD_OFFSET_SEQ];

    *p++ = in->seq_num;

    u64_to_ts_buf(in->poll_tx_ts,  p); p += MSG_TS_LEN;
    u64_to_ts_buf(in->resp_rx_ts,  p); p += MSG_TS_LEN;
    u64_to_ts_buf(in->final_tx_ts, p); p += MSG_TS_LEN;

    memcpy(p, &in->resp_rssi_q8, sizeof(int16_t)); p += sizeof(int16_t);

    *p++ = in->entry_count;
    for (int i = 0; i < in->entry_count; i++) {
        u64_to_ts_buf(in->entries[i], p);                    p += MSG_TS_LEN;
        memcpy(p, &in->entry_rssi_q8[i], sizeof(int16_t));  p += sizeof(int16_t);
        memcpy(p, &in->entry_id[i],      sizeof(uint16_t)); p += sizeof(uint16_t); // ← new
    }

    memcpy(p, &in->IMU_pitch_rad, sizeof(float)); p += sizeof(float);
    memcpy(p, &in->IMU_vel_horiz, sizeof(float)); p += sizeof(float);
    memcpy(p, &in->IMU_vel_vert,  sizeof(float)); p += sizeof(float);

    return (uint16_t)(p - buf);
}

/**
 * @brief Encode a SHARE message payload into the wire buffer.
 *
 * Wire layout:
 * @code
 * [SEQ]   1 byte
 * [SLEEP] 4 bytes  sleep_time — ms until next cycle
 * @endcode
 *
 * @param[in]  in   Populated share struct.
 * @param[out] buf  Output buffer (at least MSG_PAYLOAD_OFFSET_SHARE_SLEEP +
 *                  sizeof(uint32_t) bytes).
 * @return Total bytes written.
 */
static uint16_t msg_encode_share(const msg_share_t *in, uint8_t *buf)
{
    buf[MSG_PAYLOAD_OFFSET_SEQ] = in->seq_num;
    memcpy(&buf[MSG_PAYLOAD_OFFSET_SHARE_SLEEP], &in->sleep_time, sizeof(uint32_t));
    return MSG_PAYLOAD_OFFSET_SHARE_SLEEP + sizeof(uint32_t);
}


static uint16_t msg_encode_passive(const msg_passive_t *in, uint8_t *buf)
{
    uint8_t *p = &buf[MSG_PAYLOAD_OFFSET_SEQ];

    *p++ = in->seq_num;

    u64_to_ts_buf(in->poll_rx_ts, p);       p += MSG_TS_LEN;
    memcpy(p, &in->poll_rssi_q8, sizeof(int16_t));  p += sizeof(int16_t);

    u64_to_ts_buf(in->resp_rx_ts, p);       p += MSG_TS_LEN;
    memcpy(p, &in->resp_rssi_q8, sizeof(int16_t));  p += sizeof(int16_t);

    /* final_rx_ts removed */
    u64_to_ts_buf(in->passive_tx_ts, p); p += MSG_TS_LEN;

    *p++ = in->entry_count;
    for (int i = 0; i < in->entry_count; i++) {
        u64_to_ts_buf(in->entries[i], p);       p += MSG_TS_LEN;
        memcpy(p, &in->entry_rssi_q8[i], sizeof(int16_t)); p += sizeof(int16_t);
        memcpy(p, &in->entry_ids[i],     sizeof(uint16_t));  p += sizeof(uint16_t);
    }

    memcpy(p, &in->IMU_pitch_rad, sizeof(float)); p += sizeof(float);
    memcpy(p, &in->IMU_vel_horiz, sizeof(float)); p += sizeof(float);
    memcpy(p, &in->IMU_vel_vert,  sizeof(float)); p += sizeof(float);

    return (uint16_t)(p - buf);
}

/* ── Debug self-tests ───────────────────────────────────────────────────── */

#ifdef UWB_DEBUG

/**
 * @brief Build a fake RX frame from a TX frame for loopback testing.
 *
 * Copies the TX buffer and injects synthetic RSSI and first-path values
 * so the decoder sees realistic metadata without real hardware.
 *
 * @note May not reflect current wire layout after message format changes.
 *
 * @param[in] tx    TX frame to mirror.
 * @param[in] rssi  Synthetic total received power (Q8 fixed-point).
 * @param[in] fp    Synthetic first-path power (Q8 fixed-point).
 * @return Populated @c dwm_rx_frame_t ready for @ref msg_decode.
 */
static dwm_rx_frame_t make_rx_frame(const dwm_tx_frame_t *tx, int16_t rssi, int16_t fp)
{
    dwm_rx_frame_t rx;
    memset(&rx, 0, sizeof(rx));
    memcpy(rx.data, tx->data, tx->len);
    rx.len      = tx->len;
    rx.rssi_q8  = rssi;
    rx.fp_q8    = fp;
    return rx;
}

/** @brief Helper macro — prints a 40-bit timestamp as two 32-bit halves. */
#define MPRINT_TS(label, ts) \
    mprintf(label "0x%08lX%08lX", (uint32_t)((ts) >> 32), (uint32_t)((ts) & 0xFFFFFFFF))

/**
 * @brief Roundtrip encode/decode test for @c MSG_TYPE_SYNC.
 */
static void test_sync(void)
{
    msg_t tx_msg = {
        .type     = MSG_TYPE_SYNC,
        .sender   = 0x0011,
        .receiver = 0x0022,
        .data.sync = { .seq_num = 0xAB }
    };

    dwm_tx_frame_t tx_frame = msg_encode(&tx_msg);
    dwm_rx_frame_t rx_frame = make_rx_frame(&tx_frame, 0, 0);
    msg_t rx_msg;
    msg_decode(&rx_frame, &rx_msg);

    mprintf("SYNC IN:  seq=0x%02X\r\n",   tx_msg.data.sync.seq_num);
    mprintf("SYNC OUT: seq=0x%02X\r\n\r\n", rx_msg.data.sync.seq_num);
}

/**
 * @brief Roundtrip encode/decode test for @c MSG_TYPE_POLL.
 */
static void test_poll(void)
{
    msg_t tx_msg = {
        .type     = MSG_TYPE_POLL,
        .sender   = 0x0033,
        .receiver = 0x0044,
        .data.poll = { .seq_num = 0x05 }
    };

    dwm_tx_frame_t tx_frame = msg_encode(&tx_msg);
    dwm_rx_frame_t rx_frame = make_rx_frame(&tx_frame, -512, -256);
    msg_t rx_msg;
    msg_decode(&rx_frame, &rx_msg);

    mprintf("POLL IN:  seq=0x%02X\r\n", tx_msg.data.poll.seq_num);
    mprintf("POLL OUT: seq=0x%02X ts=", rx_msg.data.poll.seq_num);

}

/**
 * @brief Roundtrip encode/decode test for @c MSG_TYPE_RESPONSE.
 */
static void test_response(void)
{
    msg_t tx_msg = {
        .type     = MSG_TYPE_RESPONSE,
        .sender   = 0x0055,
        .receiver = 0x0066,
        .data.response = { .seq_num = 0x07 }
    };

    dwm_tx_frame_t tx_frame = msg_encode(&tx_msg);
    dwm_rx_frame_t rx_frame = make_rx_frame(&tx_frame, -300, -150);
    msg_t rx_msg;
    msg_decode(&rx_frame, &rx_msg);

    mprintf("RESP IN:  seq=0x%02X\r\n", tx_msg.data.response.seq_num);
    mprintf("RESP OUT: seq=0x%02X\r\n\r\n",
            rx_msg.data.response.seq_num);

}

/**
 * @brief Roundtrip encode/decode test for @c MSG_TYPE_FINAL.
 */
static void test_final(void)
{
    msg_t tx_msg = {
        .type     = MSG_TYPE_FINAL,
        .sender   = 0x0077,
        .receiver = 0x0088,
        .data.final = {
            .seq_num     = 0x09,
            .poll_tx_ts  = 0x0000001111111111ULL,
            .resp_rx_ts  = 0x0000002222222222ULL,
            .final_tx_ts = 0x0000003333333333ULL,
            .resp_rssi_q8 = -400,
        }
    };

    dwm_tx_frame_t tx_frame = msg_encode(&tx_msg);
    dwm_rx_frame_t rx_frame = make_rx_frame(&tx_frame, -600, -100);
    msg_t rx_msg;
    msg_decode(&rx_frame, &rx_msg);

    mprintf("FINAL IN:  seq=0x%02X\r\n", tx_msg.data.final.seq_num);
    MPRINT_TS(" poll_tx=",  tx_msg.data.final.poll_tx_ts);  mprintf("\r\n");
    MPRINT_TS(" resp_rx=",  tx_msg.data.final.resp_rx_ts);  mprintf("\r\n");
    MPRINT_TS(" final_tx=", tx_msg.data.final.final_tx_ts); mprintf("\r\n");

    mprintf("FINAL OUT: seq=0x%02X\r\n", rx_msg.data.final.seq_num);
    MPRINT_TS(" poll_tx=",  rx_msg.data.final.poll_tx_ts);  mprintf("\r\n");
    MPRINT_TS(" resp_rx=",  rx_msg.data.final.resp_rx_ts);  mprintf("\r\n");
    MPRINT_TS(" final_tx=", rx_msg.data.final.final_tx_ts); mprintf("\r\n");
}

/**
 * @brief Run all message encode/decode roundtrip tests.
 *
 * Prints results via @c mprintf. Only compiled when @c UWB_DEBUG is defined.
 *
 * @warning This function may not reflect current wire layout after
 *          message format changes — verify before relying on it.
 */
void msg_run_tests(void)
{
    mprintf("=== msg roundtrip tests ===\r\n\r\n");
    test_sync();
    test_poll();
    test_response();
    test_final();
    mprintf("=== tests done ===\r\n");
}

#endif /* UWB_DEBUG */