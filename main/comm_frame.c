#include <string.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "comm_frame.h"

static const char *TAG = "FRAME";

/* ===================== CRC8 (多项式0x07, 初值0x00) ===================== */
static uint8_t crc8_update(uint8_t crc, uint8_t byte)
{
    crc ^= byte;
    for (int b = 0; b < 8; b++)
    {
        crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x07) : (uint8_t)(crc << 1);
    }
    return crc;
}

static uint8_t crc8(const uint8_t *data, size_t len)
{
    uint8_t crc = 0x00;
    for (size_t i = 0; i < len; i++)
    {
        crc = crc8_update(crc, data[i]);
    }
    return crc;
}

/* ===================== 编帧 ===================== */

int frame_encode(const uint8_t *packet, uint8_t packet_len,
                 uint8_t *out, size_t out_size)
{
    if (packet == NULL || out == NULL ||
        packet_len == 0 || packet_len > FRAME_MAX_PAYLOAD ||
        out_size < (size_t)(packet_len + FRAME_OVERHEAD))
    {
        return -1;
    }

    out[0] = FRAME_SOF;
    out[1] = packet_len;
    memcpy(&out[2], packet, packet_len);
    // CRC覆盖 LEN+payload(不含SOF)
    out[2 + packet_len] = crc8(&out[1], (size_t)packet_len + 1);
    return packet_len + FRAME_OVERHEAD;
}

/* ===================== 字节流解析状态机 ===================== */

enum
{
    ST_WAIT_SOF = 0,
    ST_LEN,
    ST_PAYLOAD,
    ST_CRC,
};

void frame_parser_init(frame_parser_t *p, frame_packet_cb_t on_packet)
{
    frame_packet_cb_t cb = on_packet ? on_packet : p->on_packet; // 复位时可传NULL保留原回调
    memset(p, 0, sizeof(*p));
    p->state = ST_WAIT_SOF;
    p->on_packet = cb;
}

void frame_parser_feed(frame_parser_t *p, const uint8_t *data, size_t len, void *ctx)
{
    for (size_t i = 0; i < len; i++)
    {
        uint8_t byte = data[i];
        switch (p->state)
        {
        case ST_WAIT_SOF:
            if (byte == FRAME_SOF)
            {
                p->state = ST_LEN;
            }
            else
            {
                p->sync_drops++;
            }
            break;

        case ST_LEN:
            if (byte == 0 || byte > FRAME_MAX_PAYLOAD)
            {
                // 长度非法:视为误同步,重新找帧头
                p->sync_drops++;
                p->state = ST_WAIT_SOF;
            }
            else
            {
                p->expect_len = byte;
                p->got_len = 0;
                p->state = ST_PAYLOAD;
            }
            break;

        case ST_PAYLOAD:
            p->payload[p->got_len++] = byte;
            if (p->got_len >= p->expect_len)
            {
                p->state = ST_CRC;
            }
            break;

        case ST_CRC:
        {
            uint8_t crc = crc8_update(0x00, p->expect_len);
            for (int k = 0; k < p->expect_len; k++)
            {
                crc = crc8_update(crc, p->payload[k]);
            }

            if (crc == byte)
            {
                p->frames_ok++;
                if (p->on_packet)
                {
                    p->on_packet(p->payload, p->expect_len, ctx);
                }
            }
            else
            {
                p->crc_errors++;
                ESP_LOGW(TAG, "CRC error: len=%d calc=0x%02X got=0x%02X",
                         p->expect_len, crc, byte);
            }
            p->state = ST_WAIT_SOF;
            break;
        }

        default:
            p->state = ST_WAIT_SOF;
            break;
        }
    }
}