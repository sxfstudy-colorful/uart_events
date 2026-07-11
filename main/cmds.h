#ifndef CMDS_H
#define CMDS_H

#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stddef.h> // offsetof

/*
 * ============================================================
 *  cmd 应用协议(复用自 EFR32BG22 戒指项目,引擎零改动)
 *
 *  包格式: [pkt_type 1B][msg_id 4B][字段序列...]
 *  字段序列: [字段ID 1B][字段类型 1B][字段值 按类型定长]
 *  多字节值为大端序。
 *
 *  与戒指项目的差异:仅剔除了该项目的业务结构体
 *  (pray_info/vibration/lighting/firmware_version),
 *  引擎部分(字段类型/错误码/描述表/parse/write)保持一致。
 *
 *  扩展新消息三步(参考 joystick_msg.c):
 *    1. 本文件定义消息结构体 + pkt_type 枚举值
 *    2. 新建 xxx_msg.c:字段描述表 + parse/write 包装
 *    3. main.c 的分发 switch 加一个 case
 * ============================================================
 */

// 单包最大长度(沿用戒指项目GATT限制值,LoRa定点模式净载荷上限
// 为 分包长度230 - 传输头3 = 227,128在其内,统一用128便于两端复用)
#define GATT_MAX_TRANSFER_LEN 128
#define CMD_MAX_PACKET_LEN GATT_MAX_TRANSFER_LEN

// 字段类型枚举(1字节,无struct/指针类型)
typedef enum
{
    FIELD_TYPE_UINT8 = 0x01,
    FIELD_TYPE_UINT16 = 0x02,
    FIELD_TYPE_UINT32 = 0x03,
    FIELD_TYPE_FLOAT = 0x04,
    FIELD_TYPE_INT8 = 0x05,
    FIELD_TYPE_INT16 = 0x06,
    FIELD_TYPE_INT32 = 0x07,
    FIELD_TYPE_BYTE_ARR = 0x08, // 纯字节数组(无指针)
} cmd_field_type_t;

// 错误码定义
typedef enum
{
    CMD_SUCCESS = 0,
    CMD_ERR_PARAM = -1,         // 参数无效(空指针/长度非法)
    CMD_ERR_LENGTH = -2,        // 数据长度不足/超出限制
    CMD_ERR_UNKNOWN_TYPE = -3,  // 未知的包类型type
    CMD_ERR_UNKNOWN_FIELD = -4, // 未知的字段ID
    CMD_ERR_TYPE_MISMATCH = -5, // 字段类型不匹配
} cmd_status_t;

// 包类型分配(保留区间划分与戒指项目一致,便于两端语义统一)
typedef enum
{
    unknown_pkt_type = 0x00,

    // 0x01-0x1f 保留给系统使用
    heart_beat_pkt_type = 0x02,

    // 0x20-0x5f 保留给设置使用

    // 0x60-0xdf 保留给用户使用
    joystick_pkt_type = 0x60, // 摇杆状态
    // 在此追加,如 telemetry_pkt_type = 0x61(接收端回传遥测)
} cmd_pkt_type_t;

// 字段描述结构体(关联type、字段ID与本地结构体成员)
typedef struct
{
    cmd_pkt_type_t pkt_type;     // 匹配的包类型type
    uint8_t field_id;            // 字段ID
    cmd_field_type_t field_type; // 字段类型
    uint16_t data_len;           // 仅字节数组有效(固定长度)
    uint16_t struct_offset;      // 字段在本地结构体中的偏移量(offsetof)
} cmd_field_desc_t;

/* ---------- 引擎接口(与戒指项目签名完全一致) ---------- */

cmd_status_t cmd_data_write(uint8_t *output_buf, uint16_t buf_max_len, uint16_t *output_len,
                            cmd_pkt_type_t pkt_type, uint32_t msg_id, const void *src_struct, uint16_t struct_size,
                            const cmd_field_desc_t *field_desc, uint8_t desc_count);

cmd_status_t cmd_data_parse(const uint8_t *input_buf, uint16_t input_len, uint32_t *msg_id,
                            void *dest_struct, uint16_t struct_size,
                            const cmd_field_desc_t *field_desc, uint8_t desc_count);

cmd_status_t cmd_extract_pkt_type(const uint8_t *rx_buf, uint16_t rx_len, uint8_t *pkt_type);
cmd_status_t cmd_extract_pkt_msg_id(const uint8_t *rx_buf, uint16_t rx_len, uint32_t *msg_id);

/* ---------- 本项目消息定义 ---------- */

// 摇杆状态消息(msg_id兼作序号,接收端可据此检测丢包)
typedef struct
{
    int16_t x;       // X轴,归一化 -1000~+1000
    int16_t y;       // Y轴,归一化 -1000~+1000
    uint8_t buttons; // 按键位图 bit0~bit7
} joystick_msg_t;

cmd_status_t joystick_msg_parse(const uint8_t *rx_buf, uint16_t rx_len,
                                uint32_t *msg_id, joystick_msg_t *msg);
cmd_status_t joystick_msg_write(uint8_t *tx_buf, uint16_t tx_buf_len, uint16_t *tx_len,
                                uint32_t msg_id, const joystick_msg_t *msg);

// 心跳(无字段,仅type+msg_id)
cmd_status_t heartbeat_write(uint8_t *tx_buf, uint16_t tx_buf_len, uint16_t *tx_len,
                             uint32_t msg_id);

#endif // CMDS_H