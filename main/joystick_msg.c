#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stddef.h>

#include "app_log.h"

#include "cmds.h"

/*
 * ============================================================
 *  摇杆消息:字段描述表 + parse/write 包装
 *  组织模式与戒指项目的 pray_info.c 完全一致——
 *  一个消息一个.c文件,描述表静态私有,对外只暴露包装函数。
 * ============================================================
 */

static const cmd_field_desc_t _joystick_msg_field_desc[] = {
    {joystick_pkt_type, 0x01, FIELD_TYPE_INT16, 0, offsetof(joystick_msg_t, x)},
    {joystick_pkt_type, 0x02, FIELD_TYPE_INT16, 0, offsetof(joystick_msg_t, y)},
    {joystick_pkt_type, 0x03, FIELD_TYPE_UINT8, 0, offsetof(joystick_msg_t, buttons)},
};

cmd_status_t joystick_msg_parse(const uint8_t *rx_buf, uint16_t rx_len,
                                uint32_t *msg_id, joystick_msg_t *msg)
{
    return cmd_data_parse(
        rx_buf, rx_len, msg_id,
        msg, sizeof(joystick_msg_t),
        _joystick_msg_field_desc, sizeof(_joystick_msg_field_desc) / sizeof(cmd_field_desc_t));
}

cmd_status_t joystick_msg_write(uint8_t *tx_buf, uint16_t tx_buf_len, uint16_t *tx_len,
                                uint32_t msg_id, const joystick_msg_t *msg)
{
    return cmd_data_write(
        tx_buf, tx_buf_len, tx_len, joystick_pkt_type, msg_id,
        msg, sizeof(joystick_msg_t),
        _joystick_msg_field_desc, sizeof(_joystick_msg_field_desc) / sizeof(cmd_field_desc_t));
}

/* ---------- 心跳:无字段,仅 type + msg_id ---------- */

cmd_status_t heartbeat_write(uint8_t *tx_buf, uint16_t tx_buf_len, uint16_t *tx_len,
                             uint32_t msg_id)
{
    // desc_count=0:引擎只写type+msg_id共5字节,src_struct不会被访问
    return cmd_data_write(
        tx_buf, tx_buf_len, tx_len, heart_beat_pkt_type, msg_id,
        NULL, 0, NULL, 0);
}