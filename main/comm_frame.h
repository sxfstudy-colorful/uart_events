#ifndef COMM_FRAME_H
#define COMM_FRAME_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C"
{
#endif

    /*
     * ============================================================
     *  链路组帧层(极薄):补齐 GATT 与 LoRa-UART 的传输保证差异
     *
     *  cmd协议在戒指项目跑在GATT上,GATT保证"每次到达一个完整包";
     *  LoRa-UART没有这个保证:UART事件可能把一包拆成多段,也可能
     *  有杂波字节。本层用最小开销(3字节)封装cmd包:
     *
     *  帧格式: [SOF 0xA5][LEN 1B][cmd包 LEN字节][CRC8 1B]
     *    CRC8覆盖 LEN+cmd包,多项式0x07
     *
     *  接收侧字节流状态机自动组帧:凑齐完整帧且CRC通过后,
     *  把"完整cmd包"回调给上层——上层拿到的就是与GATT等价的
     *  完整包,cmd_data_parse可直接使用。
     * ============================================================
     */

#define FRAME_SOF 0xA5
#define FRAME_MAX_PAYLOAD 128 // 与 CMD_MAX_PACKET_LEN 对齐
#define FRAME_OVERHEAD 3      // SOF + LEN + CRC8

    /** @brief 完整cmd包到达回调(packet即一个完整的cmd协议包) */
    typedef void (*frame_packet_cb_t)(const uint8_t *packet, uint16_t len, void *ctx);

    typedef struct
    {
        // 内部状态,调用者不要直接访问
        int state;
        uint8_t expect_len;
        uint8_t got_len;
        uint8_t payload[FRAME_MAX_PAYLOAD];
        frame_packet_cb_t on_packet;
        // 统计
        uint32_t frames_ok;
        uint32_t crc_errors;
        uint32_t sync_drops; // 非SOF字节丢弃计数
    } frame_parser_t;

    /**
     * @brief 组帧:cmd包 → 完整帧
     * @return 帧总长,失败返回-1
     */
    int frame_encode(const uint8_t *packet, uint8_t packet_len,
                     uint8_t *out, size_t out_size);

    /** @brief 初始化/复位解析器并挂接完整包回调 */
    void frame_parser_init(frame_parser_t *p, frame_packet_cb_t on_packet);

    /** @brief 喂入一段接收字节流(容忍拆包/粘包/前置杂波) */
    void frame_parser_feed(frame_parser_t *p, const uint8_t *data, size_t len, void *ctx);

#ifdef __cplusplus
}
#endif

#endif // COMM_FRAME_H