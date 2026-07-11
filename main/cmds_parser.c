#include <stdint.h>
#include <string.h>
#include <stdbool.h>

#include "app_log.h"

#include "cmds.h"

/**
 * @豆包
 * EFR32BG22芯片，gatt传输数据解析到本地单个结构体，单次数据传输的最大长度为128B
 * 首字节为type，包类型
 * 固定4个字节的msg-id
 * 根据type判断后续字段序列
 * 字段序列：字段ID一字节，字段类型一字节、字段值排列，字段不包含struct或struct指针
 *
 * 给出解析函数和写入函数，ide使用的simplicity studio v6，语言使用的C语言
 */

/**
 * @brief 从GATT接收的数据包中提取首字节作为pkt_type
 * @param rx_buf: 蓝牙接收的原始数据缓冲区（uint8_t*）
 * @param rx_len: 接收数据的长度
 * @param pkt_type: 输出，提取到的包类型（首字节）
 * @return GATT_SUCCESS: 提取成功；其他：错误码
 */
cmd_status_t cmd_extract_pkt_type(const uint8_t *rx_buf, uint16_t rx_len, uint8_t *pkt_type)
{
    // 1. 校验输入参数有效性
    if (rx_buf == NULL || pkt_type == NULL)
    {
        return CMD_ERR_PARAM;
    }

    // 2. 校验数据长度（至少1字节才能提取首字节）
    if (rx_len < 1)
    {
        return CMD_ERR_LENGTH;
    }

    // 3. 提取首字节作为pkt_type
    *pkt_type = rx_buf[0];

    return CMD_SUCCESS;
}

cmd_status_t cmd_extract_pkt_msg_id(const uint8_t *rx_buf, uint16_t rx_len, uint32_t *msg_id)
{
    // 1. 校验输入参数有效性
    if (rx_buf == NULL || msg_id == NULL)
    {
        return CMD_ERR_PARAM;
    }

    // 2. 校验数据长度（至少1字节才能提取首字节）
    if (rx_len < 5)
    {
        return CMD_ERR_LENGTH;
    }

    // 3. 提取首字节作为pkt_type
    *msg_id = (uint32_t)rx_buf[1] << 24 |
              (uint32_t)rx_buf[2] << 16 |
              (uint32_t)rx_buf[3] << 8 |
              (uint32_t)rx_buf[4];

    return CMD_SUCCESS;
}

/**
 * @brief GATT数据解析函数（解析到单个本地结构体）
 * @param input_buf: GATT接收的原始数据（uint8_t*）
 * @param input_len: 原始数据长度（≤128）
 * @param dest_struct: 目标本地结构体（单个）
 * @param struct_size: 目标结构体总大小
 * @param field_desc: 字段描述表（关联type、字段ID与结构体成员）
 * @param desc_count: 字段描述表长度
 * @return 解析状态码
 */
cmd_status_t cmd_data_parse(const uint8_t *input_buf, uint16_t input_len, uint32_t *msg_id,
                            void *dest_struct, uint16_t struct_size,
                            const cmd_field_desc_t *field_desc, uint8_t desc_count)
{
    // 1. 基础参数校验
    if (msg_id == NULL)
    {
        return CMD_ERR_PARAM;
    }
    if (input_len < 5 || input_len > GATT_MAX_TRANSFER_LEN)
    {
        return CMD_ERR_LENGTH;
    }

    for (uint8_t i = 0; i < input_len; i++)
    {
        app_log_info("input_buf[%d] = 0x%02X" APP_LOG_NL, i, input_buf[i]);
    }

    // 2. 提取首字节：包类型type
    cmd_pkt_type_t pkt_type = input_buf[0];
    uint16_t offset = 1; // 偏移量跳过type

    cmd_extract_pkt_msg_id(input_buf, input_len, msg_id);
    app_log_info("msg_id = %lu" APP_LOG_NL, *msg_id);
    offset += 4; // 跳过msg_id

    // 3. 循环解析字段序列
    while (offset < input_len)
    {
        // 校验剩余长度（至少包含字段ID+字段类型：2字节）
        if (offset + 1 >= input_len)
        {
            return CMD_ERR_LENGTH;
        }

        uint8_t field_id = input_buf[offset++];            // 字段ID
        cmd_field_type_t field_type = input_buf[offset++]; // 字段类型

        app_log_info("field_id = 0x%02X, field_type = 0x%02X" APP_LOG_NL,
                     field_id, field_type);

        // 4. 查找匹配的字段描述（type+field_id双匹配）
        const cmd_field_desc_t *desc = NULL;
        for (uint8_t i = 0; i < desc_count; i++)
        {
            if (field_desc[i].pkt_type == pkt_type && field_desc[i].field_id == field_id)
            {
                desc = &field_desc[i];
                break;
            }
        }
        if (desc == NULL)
        {
            return CMD_ERR_UNKNOWN_FIELD; // 无匹配的字段ID
        }

        // 5. 校验字段类型匹配
        if (desc->field_type != field_type)
        {
            return CMD_ERR_TYPE_MISMATCH;
        }

        app_log_info("field_id = 0x%02X, field_type = 0x%02X, struct_offset = %d" APP_LOG_NL,
                     field_id, field_type, desc->struct_offset);

        // 6. 计算结构体成员地址（通过偏移量，避免指针直接操作）
        uint8_t *field_ptr = (uint8_t *)dest_struct + desc->struct_offset;
        if ((uint16_t)(field_ptr - (uint8_t *)dest_struct) >= struct_size)
        {
            return CMD_ERR_PARAM; // 偏移量超出结构体大小
        }

        // 7. 根据字段类型解析值到结构体
        switch (field_type)
        {
        case FIELD_TYPE_UINT8:
            if (offset >= input_len)
                return CMD_ERR_LENGTH;
            *(uint8_t *)field_ptr = input_buf[offset++];
            break;

        case FIELD_TYPE_UINT16:
            if (offset + 1 >= input_len)
                return CMD_ERR_LENGTH;
            // 大端序解析（蓝牙标准）
            *(uint16_t *)field_ptr = (uint16_t)(input_buf[offset] << 8) | input_buf[offset + 1];
            offset += 2;
            break;

        case FIELD_TYPE_UINT32:
            if (offset + 3 >= input_len)
                return CMD_ERR_LENGTH;
            *(uint32_t *)field_ptr = (uint32_t)(input_buf[offset] << 24) |
                                     (uint32_t)(input_buf[offset + 1] << 16) |
                                     (uint32_t)(input_buf[offset + 2] << 8) |
                                     input_buf[offset + 3];
            offset += 4;
            app_log_info("uint32_t value = 0x%08lX" APP_LOG_NL, *(uint32_t *)field_ptr);
            break;

        case FIELD_TYPE_FLOAT:
            if (offset + 3 >= input_len)
                return CMD_ERR_LENGTH;
            union
            {
                uint32_t u32;
                float f;
            } float_conv;
            float_conv.u32 = (uint32_t)(input_buf[offset] << 24) |
                             (uint32_t)(input_buf[offset + 1] << 16) |
                             (uint32_t)(input_buf[offset + 2] << 8) |
                             input_buf[offset + 3];
            *(float *)field_ptr = float_conv.f;
            offset += 4;
            break;

        case FIELD_TYPE_INT8:
            if (offset >= input_len)
                return CMD_ERR_LENGTH;
            *(int8_t *)field_ptr = input_buf[offset++];
            break;

        case FIELD_TYPE_INT16:
            if (offset + 1 >= input_len)
                return CMD_ERR_LENGTH;
            *(int16_t *)field_ptr = (int16_t)(input_buf[offset] << 8) | input_buf[offset + 1];
            offset += 2;
            break;

        case FIELD_TYPE_INT32:
            if (offset + 3 >= input_len)
                return CMD_ERR_LENGTH;
            *(int32_t *)field_ptr = (int32_t)(input_buf[offset] << 24) |
                                    (int32_t)(input_buf[offset + 1] << 16) |
                                    (int32_t)(input_buf[offset + 2] << 8) |
                                    input_buf[offset + 3];
            offset += 4;
            break;

        case FIELD_TYPE_BYTE_ARR:
            if (offset + desc->data_len > input_len)
                return CMD_ERR_LENGTH;
            memcpy(field_ptr, &input_buf[offset], desc->data_len);
            offset += desc->data_len;
            break;

        default:
            return CMD_ERR_TYPE_MISMATCH;
        }
    }

    return CMD_SUCCESS;
}

/**
 * @brief GATT数据写入函数（单个本地结构体打包为GATT发送数据）
 * @param output_buf: 输出缓冲区（uint8_t*，用于GATT发送）
 * @param buf_max_len: 缓冲区最大长度（≤128）
 * @param output_len: 输出，实际打包后的长度
 * @param pkt_type: 包类型（首字节）
 * @param src_struct: 源本地结构体（单个）
 * @param struct_size: 源结构体总大小
 * @param field_desc: 字段描述表（关联type、字段ID与结构体成员）
 * @param desc_count: 字段描述表长度
 * @return 写入状态码
 */
cmd_status_t cmd_data_write(uint8_t *output_buf, uint16_t buf_max_len, uint16_t *output_len,
                            cmd_pkt_type_t pkt_type, uint32_t msg_id, const void *src_struct, uint16_t struct_size,
                            const cmd_field_desc_t *field_desc, uint8_t desc_count)
{
    // 1. 基础参数校验
    if (output_buf == NULL || output_len == NULL)
    {
        return CMD_ERR_PARAM;
    }
    if (buf_max_len < 5 || buf_max_len > GATT_MAX_TRANSFER_LEN)
    {
        return CMD_ERR_LENGTH;
    }

    // 2. 写入首字节：包类型type
    output_buf[0] = pkt_type;
    uint16_t offset = 1;

    // 2. 写入msg_id（4字节）
    output_buf[offset++] = (uint8_t)(msg_id >> 24);
    output_buf[offset++] = (uint8_t)(msg_id >> 16);
    output_buf[offset++] = (uint8_t)(msg_id >> 8);
    output_buf[offset++] = (uint8_t)(msg_id & 0xFF);

    // 3. 循环写入匹配type的字段序列
    for (uint8_t i = 0; i < desc_count; i++)
    {
        const cmd_field_desc_t *desc = &field_desc[i];
        // 只处理当前type匹配的字段
        if (desc->pkt_type != pkt_type)
        {
            continue;
        }

        // 4. 计算当前字段需要的长度（ID+类型+值）
        uint16_t need_len = 2; // ID(1) + 类型(1)
        switch (desc->field_type)
        {
        case FIELD_TYPE_UINT8:
        case FIELD_TYPE_INT8:
            need_len += 1;
            break;
        case FIELD_TYPE_UINT16:
        case FIELD_TYPE_INT16:
            need_len += 2;
            break;
        case FIELD_TYPE_UINT32:
        case FIELD_TYPE_INT32:
        case FIELD_TYPE_FLOAT:
            need_len += 4;
            break;
        case FIELD_TYPE_BYTE_ARR:
            need_len += desc->data_len;
            break;
        default:
            return CMD_ERR_TYPE_MISMATCH;
        }

        // 5. 校验缓冲区剩余空间
        if (offset + need_len > buf_max_len)
        {
            return CMD_ERR_LENGTH;
        }

        // 6. 计算结构体成员地址（偏移量）
        const uint8_t *field_ptr = (const uint8_t *)src_struct + desc->struct_offset;
        if ((uint16_t)(field_ptr - (const uint8_t *)src_struct) >= struct_size)
        {
            return CMD_ERR_PARAM;
        }

        // 7. 写入字段ID和类型
        output_buf[offset++] = desc->field_id;
        output_buf[offset++] = desc->field_type;

        // 8. 写入字段值（大端序）
        switch (desc->field_type)
        {
        case FIELD_TYPE_UINT8:
            output_buf[offset++] = *(const uint8_t *)field_ptr;
            break;

        case FIELD_TYPE_UINT16:
            output_buf[offset++] = (uint8_t)((*(const uint16_t *)field_ptr) >> 8);
            output_buf[offset++] = (uint8_t)(*(const uint16_t *)field_ptr & 0xFF);
            break;

        case FIELD_TYPE_UINT32:
            output_buf[offset++] = (uint8_t)((*(const uint32_t *)field_ptr) >> 24);
            output_buf[offset++] = (uint8_t)((*(const uint32_t *)field_ptr) >> 16 & 0xFF);
            output_buf[offset++] = (uint8_t)((*(const uint32_t *)field_ptr) >> 8 & 0xFF);
            output_buf[offset++] = (uint8_t)(*(const uint32_t *)field_ptr & 0xFF);
            break;

        case FIELD_TYPE_FLOAT:
            union
            {
                uint32_t u32;
                float f;
            } float_conv;
            float_conv.f = *(const float *)field_ptr;
            output_buf[offset++] = (uint8_t)(float_conv.u32 >> 24);
            output_buf[offset++] = (uint8_t)(float_conv.u32 >> 16 & 0xFF);
            output_buf[offset++] = (uint8_t)(float_conv.u32 >> 8 & 0xFF);
            output_buf[offset++] = (uint8_t)(float_conv.u32 & 0xFF);
            break;

        case FIELD_TYPE_INT8:
            output_buf[offset++] = *(const int8_t *)field_ptr;
            break;

        case FIELD_TYPE_INT16:
            output_buf[offset++] = (uint8_t)((*(const int16_t *)field_ptr) >> 8);
            output_buf[offset++] = (uint8_t)(*(const int16_t *)field_ptr & 0xFF);
            break;

        case FIELD_TYPE_INT32:
            output_buf[offset++] = (uint8_t)((*(const int32_t *)field_ptr) >> 24);
            output_buf[offset++] = (uint8_t)((*(const int32_t *)field_ptr) >> 16 & 0xFF);
            output_buf[offset++] = (uint8_t)((*(const int32_t *)field_ptr) >> 8 & 0xFF);
            output_buf[offset++] = (uint8_t)(*(const int32_t *)field_ptr & 0xFF);
            break;

        case FIELD_TYPE_BYTE_ARR:
            memcpy(&output_buf[offset], field_ptr, desc->data_len);
            offset += desc->data_len;
            break;

        default:
            return CMD_ERR_TYPE_MISMATCH;
        }
    }

    // 9. 设置实际打包长度
    *output_len = offset;
    return CMD_SUCCESS;
}