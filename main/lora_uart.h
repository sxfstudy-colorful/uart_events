#ifndef LORA_UART_H
#define LORA_UART_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "board_config.h" // AUX_CONNECTED 等硬件形态宏,驱动层必须可见

#ifdef __cplusplus
extern "C"
{
#endif

    /*
     * ============================================================
     *  DX-LR41 驱动层:AT配置 + 原始字节收发
     *  本层只认字节和LoRa传输头(地址+信道),不理解业务数据含义。
     * ============================================================
     */

#define LORA_UART_BUF_SIZE 1024

/* ---------- AT 时序参数 ---------- */
#define AUX_WAIT_TIMEOUT 3000
#define TX_FIXED_DELAY_MS 100
#define RESET_FIXED_DELAY_MS 1500
#define PLUS_GUARD_BEFORE_MS 1200
#define PLUS_RESP_TIMEOUT_MS 2000
#define PLUS_RETRY_BASE_MS 1500
#define AT_RETRY_MAX 3
#define CONFIG_VERIFY_ATTEMPTS 2

    typedef enum
    {
        AT_STATE_UNKNOWN = 0,
        AT_STATE_TRANSMIT, // 传输模式(上电默认)
        AT_STATE_COMMAND   // AT命令模式
    } at_mode_state_t;

    /* 前置声明:回调签名要引用本结构体自身 */
    struct lora_module_s;

    /**
     * @brief 接收数据回调(已剥离LoRa传输头,data为纯业务字节流)
     * 注意:data可能是不完整/粘连的帧片段,上层协议解析器负责组帧
     */
    typedef void (*lora_rx_callback_t)(const struct lora_module_s *mod,
                                       const uint8_t *data, size_t len);

    typedef struct lora_module_s
    {
        /* ---- 硬件描述 ---- */
        const char *name;
        uart_port_t uart_num;
        gpio_num_t tx_pin;
        gpio_num_t rx_pin;
        gpio_num_t aux_pin;
        QueueHandle_t *evt_queue;

        /* ---- 模组自身参数(AT配置写入模组) ---- */
        int mode;       // 传输模式 0/1/2
        int level;      // 速率等级
        uint8_t addr_h; // 本模组地址(定点模式下即"收件地址")
        uint8_t addr_l;
        uint8_t channel;    // 本模组监听信道
        bool enable_verify; // 配置后是否回读验证

        /* ---- 发送目标(组包用,不写入模组) ---- */
        uint8_t target_addr_h;
        uint8_t target_addr_l;
        uint8_t target_channel;

        /* ---- 运行时状态(驱动内部维护,调用者只读) ---- */
        at_mode_state_t at_state;

        /* ---- 上层挂接 ---- */
        lora_rx_callback_t rx_callback;
    } lora_module_t;

    /* ---------- 初始化 ---------- */
    void lora_uart_init(lora_module_t *mod, QueueHandle_t *evt_queue);
    void aux_gpio_init(gpio_num_t aux_pin);

    /* ---------- AT 配置流程 ---------- */
    bool configure_lora_module(lora_module_t *mod);
    bool verify_module_config(lora_module_t *mod);
    bool configure_and_verify(lora_module_t *mod);

    /* ---------- AT 原语(一般无需直接调用) ---------- */
    esp_err_t send_plus_plus_plus(lora_module_t *mod, at_mode_state_t *new_state);
    esp_err_t send_plus_plus_plus_retry(lora_module_t *mod,
                                        at_mode_state_t *new_state, uint8_t retry_max);
    esp_err_t send_at_config_cmd(lora_module_t *mod, const char *cmd, uint32_t timeout_ms);
    esp_err_t send_at_config_cmd_retry(lora_module_t *mod, const char *cmd,
                                       uint32_t timeout_ms, uint8_t retry_max);
    esp_err_t send_at_query_cmd(lora_module_t *mod, const char *cmd,
                                const char *key, int *value, uint32_t timeout_ms);
    void send_at_reset_no_wait(lora_module_t *mod);

    /* ---------- 数据收发 ---------- */

    /**
     * @brief 发送二进制数据(核心发送接口,摇杆等二进制协议帧必须走这里)
     * 内部按模组模式自动加LoRa传输头(定点=目标地址+信道)
     */
    esp_err_t lora_send_bytes(lora_module_t *mod, const uint8_t *data, size_t len);

    /** @brief 发送C字符串(调试便捷封装,内部转调lora_send_bytes) */
    esp_err_t lora_send_data(lora_module_t *mod, const char *data);

    /* ---------- 工具 ---------- */
    bool wait_lora_idle(const lora_module_t *mod, uint32_t timeout_ms);
    bool wait_module_reset_complete(const lora_module_t *mod, uint32_t timeout_ms);
    void clear_uart_buffer(uart_port_t uart);
    char *get_response_string(uint8_t *buf, size_t len);

#ifdef __cplusplus
}
#endif

#endif // LORA_UART_H