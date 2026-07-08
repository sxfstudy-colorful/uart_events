#ifndef LORA_UART_H
#define LORA_UART_H

#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "driver/uart.h"
#include "driver/gpio.h"

#ifdef __cplusplus
extern "C"
{
#endif

#define LORA_UART_BUF_SIZE 1024

#define LORA_TARGET_ADDR_H 0x00
#define LORA_TARGET_ADDR_L 0x01
#define LORA_TARGET_CH 0x01

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
        AT_STATE_TRANSMIT,
        AT_STATE_COMMAND
    } at_mode_state_t;

    typedef void (*lora_rx_callback_t)(const struct lora_module_s *mod, const uint8_t *data, size_t len);

    typedef struct lora_module_s
    {
        const char *name;
        uart_port_t uart_num;
        gpio_num_t tx_pin;
        gpio_num_t rx_pin;
        gpio_num_t aux_pin;
        QueueHandle_t *evt_queue;
        bool is_rx;
        int mode;
        int level;
        bool enable_verify;
        uint8_t addr_h;
        uint8_t addr_l;
        uint8_t channel;
        uint8_t target_addr_h;
        uint8_t target_addr_l;
        uint8_t target_channel;
        lora_rx_callback_t rx_callback;
    } lora_module_t;

    bool wait_lora_idle(const lora_module_t *mod, uint32_t timeout_ms);

    bool wait_module_reset_complete(const lora_module_t *mod, uint32_t timeout_ms);

    void lora_uart_init(const lora_module_t *mod, QueueHandle_t *evt_queue);

    void aux_gpio_init(gpio_num_t aux_pin);

    void clear_uart_buffer(uart_port_t uart);

    char *get_response_string(uint8_t *buf, size_t len);

    esp_err_t send_plus_plus_plus(const lora_module_t *mod, at_mode_state_t *new_state);

    esp_err_t send_at_config_cmd(const lora_module_t *mod, const char *cmd, uint32_t timeout_ms);

    esp_err_t send_at_query_cmd(const lora_module_t *mod, const char *cmd,
                                const char *key, int *value, uint32_t timeout_ms);

    esp_err_t send_plus_plus_plus_retry(const lora_module_t *mod,
                                        at_mode_state_t *new_state, uint8_t retry_max);

    esp_err_t send_at_config_cmd_retry(const lora_module_t *mod,
                                       const char *cmd, uint32_t timeout_ms, uint8_t retry_max);

    void send_at_reset_no_wait(const lora_module_t *mod);

    bool configure_lora_module(const lora_module_t *mod);

    bool verify_module_config(const lora_module_t *mod, int expect_mode, int expect_level);

    bool configure_and_verify(const lora_module_t *mod);

    void build_lora_packet(const lora_module_t *mod, const char *src, uint8_t *dst);

    esp_err_t lora_send_data(const lora_module_t *mod, const char *data);

#ifdef __cplusplus
}
#endif

#endif
