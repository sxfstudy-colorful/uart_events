#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "lora_uart.h"

static const char *TAG = "LORA_UART";

/* ===================== AUX / 复位等待 ===================== */

bool wait_lora_idle(const lora_module_t *mod, uint32_t timeout_ms)
{
#if !AUX_CONNECTED
    (void)mod;
    (void)timeout_ms;
    vTaskDelay(pdMS_TO_TICKS(TX_FIXED_DELAY_MS));
    return true;
#else
    uint32_t tick_start = xTaskGetTickCount();
    const uint32_t tick_timeout = pdMS_TO_TICKS(timeout_ms);
    while (gpio_get_level(mod->aux_pin) == 1)
    {
        vTaskDelay(pdMS_TO_TICKS(10));
        if ((xTaskGetTickCount() - tick_start) >= tick_timeout)
        {
            ESP_LOGE(TAG, "%s: AUX wait timeout!", mod->name);
            return false;
        }
    }
    return true;
#endif
}

bool wait_module_reset_complete(const lora_module_t *mod, uint32_t timeout_ms)
{
#if !AUX_CONNECTED
    (void)mod;
    (void)timeout_ms;
    ESP_LOGI(TAG, "%s: AUX not connected, fixed wait %dms", mod->name, RESET_FIXED_DELAY_MS);
    vTaskDelay(pdMS_TO_TICKS(RESET_FIXED_DELAY_MS));
    return true;
#else
    ESP_LOGI(TAG, "%s: Waiting reset complete (AUX high→low)...", mod->name);
    uint32_t tick_start = xTaskGetTickCount();
    const uint32_t tick_timeout = pdMS_TO_TICKS(timeout_ms);

    while (gpio_get_level(mod->aux_pin) == 0)
    {
        vTaskDelay(pdMS_TO_TICKS(10));
        if ((xTaskGetTickCount() - tick_start) >= tick_timeout)
        {
            ESP_LOGW(TAG, "%s: Timeout waiting AUX high", mod->name);
            return false;
        }
    }
    tick_start = xTaskGetTickCount();
    while (gpio_get_level(mod->aux_pin) == 1)
    {
        vTaskDelay(pdMS_TO_TICKS(10));
        if ((xTaskGetTickCount() - tick_start) >= tick_timeout)
        {
            ESP_LOGW(TAG, "%s: Timeout waiting AUX low", mod->name);
            return false;
        }
    }
    ESP_LOGI(TAG, "%s: Reset complete", mod->name);
    return true;
#endif
}

/* ===================== 初始化 ===================== */

void lora_uart_init(lora_module_t *mod, QueueHandle_t *evt_queue)
{
    uart_config_t uart_cfg = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    uart_driver_install(mod->uart_num, LORA_UART_BUF_SIZE * 2, LORA_UART_BUF_SIZE * 2,
                        20, evt_queue, 0);
    uart_param_config(mod->uart_num, &uart_cfg);
    uart_set_pin(mod->uart_num, mod->tx_pin, mod->rx_pin,
                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    mod->at_state = AT_STATE_TRANSMIT; // 上电默认传输模式
}

void aux_gpio_init(gpio_num_t aux_pin)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = 1ULL << aux_pin,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
}

/* ===================== 工具 ===================== */

void clear_uart_buffer(uart_port_t uart)
{
    uint8_t tmp[32];
    int len;
    do
    {
        len = uart_read_bytes(uart, tmp, sizeof(tmp), pdMS_TO_TICKS(10));
    } while (len > 0);
}

char *get_response_string(uint8_t *buf, size_t len)
{
    (void)len;
    char *p = (char *)buf;
    while (*p == '\r' || *p == '\n')
        p++;
    size_t slen = strlen(p);
    if (slen == 0)
        return p;
    char *end = p + slen - 1;
    while (end > p && (*end == '\r' || *end == '\n'))
    {
        *end = '\0';
        end--;
    }
    return p;
}

/* ===================== AT 原语 ===================== */

esp_err_t send_plus_plus_plus(lora_module_t *mod, at_mode_state_t *new_state)
{
    uint8_t recv_buf[128] = {0};
    int recv_len;
    size_t total = 0;
    esp_err_t ret = ESP_FAIL;
    uart_port_t uart = mod->uart_num;

    clear_uart_buffer(uart);
    vTaskDelay(pdMS_TO_TICKS(PLUS_GUARD_BEFORE_MS));

    ESP_LOGI(TAG, "%s: Send +++", mod->name);
    uart_write_bytes(uart, "+++\r\n", 5);
    uart_wait_tx_done(uart, pdMS_TO_TICKS(50));

    recv_len = uart_read_bytes(uart, recv_buf, sizeof(recv_buf) - 1,
                               pdMS_TO_TICKS(PLUS_RESP_TIMEOUT_MS));

    if (recv_len <= 0)
    {
        ESP_LOGW(TAG, "%s: No response to +++", mod->name);
        return ESP_FAIL;
    }

    total = recv_len;
    recv_buf[total] = '\0';
    char *resp = get_response_string(recv_buf, total);
    ESP_LOGI(TAG, "%s: +++ Response: [%s]", mod->name, resp);

    if (strstr(resp, "Entry AT") != NULL)
    {
        *new_state = AT_STATE_COMMAND;
        ret = ESP_OK;
        ESP_LOGI(TAG, "%s: → Entered AT mode", mod->name);
    }
    else if (strstr(resp, "Exit AT") != NULL)
    {
        *new_state = AT_STATE_TRANSMIT;
        ret = ESP_OK;
        ESP_LOGI(TAG, "%s: → Exited AT mode", mod->name);
        ESP_LOGI(TAG, "%s: Draining remaining responses...", mod->name);
        for (;;)
        {
            memset(recv_buf, 0, sizeof(recv_buf));
            recv_len = uart_read_bytes(uart, recv_buf, sizeof(recv_buf) - 1,
                                       pdMS_TO_TICKS(500));
            if (recv_len <= 0)
                break;
            recv_buf[recv_len] = '\0';
            ESP_LOGI(TAG, "%s: Extra resp: [%s]", mod->name,
                     get_response_string(recv_buf, recv_len));
        }
        ESP_LOGI(TAG, "%s: → UART buffer cleared", mod->name);
    }
    else if (strstr(resp, "Power On") != NULL || strstr(resp, "Power on") != NULL)
    {
        *new_state = AT_STATE_TRANSMIT;
        ret = ESP_OK;
        ESP_LOGI(TAG, "%s: → Module in transmit mode (power on)", mod->name);
    }
    else if (strstr(resp, "OK") != NULL)
    {
        if (mod->at_state == AT_STATE_COMMAND)
        {
            *new_state = AT_STATE_TRANSMIT;
            ESP_LOGI(TAG, "%s: → Exited AT mode (OK)", mod->name);
            for (;;)
            {
                memset(recv_buf, 0, sizeof(recv_buf));
                recv_len = uart_read_bytes(uart, recv_buf, sizeof(recv_buf) - 1,
                                           pdMS_TO_TICKS(500));
                if (recv_len <= 0)
                    break;
                recv_buf[recv_len] = '\0';
                ESP_LOGI(TAG, "%s: Extra resp: [%s]", mod->name,
                         get_response_string(recv_buf, recv_len));
            }
            ESP_LOGI(TAG, "%s: → UART buffer cleared", mod->name);
        }
        else
        {
            *new_state = AT_STATE_COMMAND;
            ESP_LOGI(TAG, "%s: → Entered AT mode (OK)", mod->name);
        }
        ret = ESP_OK;
    }
    else
    {
        ESP_LOGW(TAG, "%s: Unknown +++ response: %s", mod->name, resp);
        ret = ESP_FAIL;
    }

    if (ret == ESP_OK)
    {
        mod->at_state = *new_state;
    }
    return ret;
}

esp_err_t send_at_config_cmd(lora_module_t *mod, const char *cmd, uint32_t timeout_ms)
{
    uint8_t recv_buf[128] = {0};
    size_t total = 0;
    int recv_len;
    char full_cmd[32];
    uart_port_t uart = mod->uart_num;

    if (mod->at_state != AT_STATE_COMMAND)
    {
        ESP_LOGE(TAG, "%s: Not in AT mode, cmd: %s", mod->name, cmd);
        return ESP_FAIL;
    }

    clear_uart_buffer(uart);

    snprintf(full_cmd, sizeof(full_cmd), "%s\r\n", cmd);
    ESP_LOGI(TAG, "%s: Send AT: %s", mod->name, cmd);
    uart_write_bytes(uart, full_cmd, strlen(full_cmd));
    uart_wait_tx_done(uart, pdMS_TO_TICKS(50));

    uint32_t tick_start = xTaskGetTickCount();
    const uint32_t tick_timeout = pdMS_TO_TICKS(timeout_ms);
    while ((xTaskGetTickCount() - tick_start) < tick_timeout &&
           total < sizeof(recv_buf) - 1)
    {
        recv_len = uart_read_bytes(uart, recv_buf + total,
                                   sizeof(recv_buf) - 1 - total,
                                   pdMS_TO_TICKS(100));
        if (recv_len > 0)
        {
            total += recv_len;
            recv_buf[total] = '\0';
            // 兼容手册中的EEROR拼写
            if (strstr((char *)recv_buf, "OK") != NULL ||
                strstr((char *)recv_buf, "ERROR") != NULL ||
                strstr((char *)recv_buf, "EEROR") != NULL)
            {
                break;
            }
        }
    }

    if (total > 0)
    {
        char *resp = get_response_string(recv_buf, total);
        ESP_LOGI(TAG, "%s: AT Resp: [%s]", mod->name, resp);
        if (strstr(resp, "OK") != NULL)
        {
            return ESP_OK;
        }
        if (strstr(resp, "ERROR") != NULL || strstr(resp, "EEROR") != NULL)
        {
            ESP_LOGE(TAG, "%s: AT error: %s", mod->name, resp);
            return ESP_FAIL;
        }
        // 兜底:参数回显已到但OK丢失(如被复位截断),回显正确即已被接受
        if (strstr(resp, "+MODE=") != NULL || strstr(resp, "+LEVEL=") != NULL ||
            strstr(resp, "+MAC=") != NULL || strstr(resp, "+CHANNEL=") != NULL)
        {
            ESP_LOGW(TAG, "%s: No OK but got echo, treat as success", mod->name);
            return ESP_OK;
        }
    }

    ESP_LOGW(TAG, "%s: Command %s timeout", mod->name, cmd);
    return ESP_FAIL;
}

esp_err_t send_at_query_cmd(lora_module_t *mod, const char *cmd,
                            const char *key, int *value, uint32_t timeout_ms)
{
    uint8_t recv_buf[128] = {0};
    size_t total = 0;
    int recv_len;
    char full_cmd[32];
    uart_port_t uart = mod->uart_num;

    if (mod->at_state != AT_STATE_COMMAND)
    {
        ESP_LOGE(TAG, "%s: Not in AT mode, cannot query: %s", mod->name, cmd);
        return ESP_FAIL;
    }

    clear_uart_buffer(uart);

    snprintf(full_cmd, sizeof(full_cmd), "%s\r\n", cmd);
    ESP_LOGI(TAG, "%s: Send AT query: %s", mod->name, cmd);
    uart_write_bytes(uart, full_cmd, strlen(full_cmd));
    uart_wait_tx_done(uart, pdMS_TO_TICKS(50));

    uint32_t tick_start = xTaskGetTickCount();
    const uint32_t tick_timeout = pdMS_TO_TICKS(timeout_ms);
    while ((xTaskGetTickCount() - tick_start) < tick_timeout &&
           total < sizeof(recv_buf) - 1)
    {
        recv_len = uart_read_bytes(uart, recv_buf + total,
                                   sizeof(recv_buf) - 1 - total,
                                   pdMS_TO_TICKS(100));
        if (recv_len <= 0)
            continue;
        total += recv_len;
        recv_buf[total] = '\0';

        if (strstr((char *)recv_buf, "ERROR") != NULL ||
            strstr((char *)recv_buf, "EEROR") != NULL)
        {
            ESP_LOGE(TAG, "%s: Query got ERROR", mod->name);
            return ESP_FAIL;
        }

        char *p = strstr((char *)recv_buf, key);
        if (p != NULL)
        {
            char *digit = p + strlen(key);
            // 十六进制解析(CHANNEL等应答为hex;MODE/LEVEL单数字hex==dec,统一hex安全)
            if (isxdigit((unsigned char)*digit))
            {
                *value = (int)strtol(digit, NULL, 16);
                ESP_LOGI(TAG, "%s: %s → %s%X", mod->name, cmd, key, *value);
                return ESP_OK;
            }
        }
    }

    if (total > 0)
    {
        ESP_LOGW(TAG, "%s: Query incomplete: [%s]", mod->name,
                 get_response_string(recv_buf, total));
    }
    else
    {
        ESP_LOGW(TAG, "%s: Query timeout", mod->name);
    }
    return ESP_FAIL;
}

esp_err_t send_plus_plus_plus_retry(lora_module_t *mod,
                                    at_mode_state_t *new_state, uint8_t retry_max)
{
    esp_err_t ret;
    for (uint8_t i = 0; i < retry_max; i++)
    {
        ret = send_plus_plus_plus(mod, new_state);
        if (ret == ESP_OK && *new_state == AT_STATE_COMMAND)
        {
            return ESP_OK;
        }
        ESP_LOGW(TAG, "%s: +++ retry %d/%d", mod->name, i + 1, retry_max);
        clear_uart_buffer(mod->uart_num);
        vTaskDelay(pdMS_TO_TICKS(PLUS_RETRY_BASE_MS * (i + 1)));
    }
    ESP_LOGE(TAG, "%s: +++ failed after %d retries", mod->name, retry_max);
    return ESP_FAIL;
}

esp_err_t send_at_config_cmd_retry(lora_module_t *mod, const char *cmd,
                                   uint32_t timeout_ms, uint8_t retry_max)
{
    esp_err_t ret;
    for (uint8_t i = 0; i < retry_max; i++)
    {
        ret = send_at_config_cmd(mod, cmd, timeout_ms);
        if (ret == ESP_OK)
            return ESP_OK;
        ESP_LOGW(TAG, "%s: AT cmd retry %d/%d: %s", mod->name, i + 1, retry_max, cmd);
        clear_uart_buffer(mod->uart_num);
        vTaskDelay(pdMS_TO_TICKS(200 * (i + 1)));
    }
    ESP_LOGE(TAG, "%s: AT cmd %s failed", mod->name, cmd);
    return ESP_FAIL;
}

void send_at_reset_no_wait(lora_module_t *mod)
{
    uart_port_t uart = mod->uart_num;
    ESP_LOGI(TAG, "%s: Send AT+RESET (no wait)", mod->name);
    uart_write_bytes(uart, "AT+RESET\r\n", 10);
    uart_wait_tx_done(uart, pdMS_TO_TICKS(50));
    mod->at_state = AT_STATE_TRANSMIT;
    wait_module_reset_complete(mod, 3000);
}

/* ===================== 配置与验证 ===================== */

bool configure_lora_module(lora_module_t *mod)
{
    at_mode_state_t new_state;

    ESP_LOGI(TAG, "==== Config %s ====", mod->name);

#if AUX_CONNECTED
    ESP_LOGI(TAG, "%s: AUX level: %d", mod->name, gpio_get_level(mod->aux_pin));
#endif

    mod->at_state = AT_STATE_TRANSMIT;
    if (send_plus_plus_plus_retry(mod, &new_state, AT_RETRY_MAX) != ESP_OK)
    {
        ESP_LOGE(TAG, "%s: Failed to enter AT mode", mod->name);
        return false;
    }

    vTaskDelay(pdMS_TO_TICKS(100));

    char cmd_buf[24];
    bool config_ok = true;

    // 传输模式
    snprintf(cmd_buf, sizeof(cmd_buf), "AT+MODE%d", mod->mode);
    config_ok &= (send_at_config_cmd_retry(mod, cmd_buf, 800, AT_RETRY_MAX) == ESP_OK);
    vTaskDelay(pdMS_TO_TICKS(50));

    // 速率等级
    snprintf(cmd_buf, sizeof(cmd_buf), "AT+LEVEL%d", mod->level);
    config_ok &= (send_at_config_cmd_retry(mod, cmd_buf, 800, AT_RETRY_MAX) == ESP_OK);
    vTaskDelay(pdMS_TO_TICKS(50));

    // 工作信道:所有模式都必须配置!
    // 定点模式下信道决定本模组的监听频点;此前仅透明模式配信道,
    // 同桌调试因距离过近碰巧跨信道收到(手册5.3.6备注3),拉开距离即断链
    snprintf(cmd_buf, sizeof(cmd_buf), "AT+CHANNEL%02X", mod->channel);
    config_ok &= (send_at_config_cmd_retry(mod, cmd_buf, 800, AT_RETRY_MAX) == ESP_OK);
    vTaskDelay(pdMS_TO_TICKS(50));

    // 设备地址:定点/广播模式需要
    if (mod->mode != 0)
    {
        snprintf(cmd_buf, sizeof(cmd_buf), "AT+MAC%02X,%02X", mod->addr_h, mod->addr_l);
        config_ok &= (send_at_config_cmd_retry(mod, cmd_buf, 800, AT_RETRY_MAX) == ESP_OK);
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    if (!config_ok)
    {
        ESP_LOGE(TAG, "%s: Config failed", mod->name);
        return false;
    }

    // 手册规则:参数设置后必须重启生效
    ESP_LOGI(TAG, "%s: Send AT+RESET to apply config...", mod->name);
    send_at_reset_no_wait(mod);

    ESP_LOGI(TAG, "%s: Config SUCCESS", mod->name);
    return true;
}

bool verify_module_config(lora_module_t *mod)
{
    at_mode_state_t new_state;
    int mode_val = -1, level_val = -1, chan_val = -1;
    bool ok = true;

    ESP_LOGI(TAG, "---- Verify %s ----", mod->name);

    mod->at_state = AT_STATE_TRANSMIT;
    if (send_plus_plus_plus_retry(mod, &new_state, 2) != ESP_OK)
    {
        ESP_LOGW(TAG, "%s: Cannot enter AT mode for verify", mod->name);
        return false;
    }

    if (send_at_query_cmd(mod, "AT+MODE", "+MODE=", &mode_val, 800) != ESP_OK ||
        mode_val != mod->mode)
    {
        ESP_LOGE(TAG, "%s: MODE=%d, expect %d", mod->name, mode_val, mod->mode);
        ok = false;
    }
    vTaskDelay(pdMS_TO_TICKS(50));

    if (send_at_query_cmd(mod, "AT+LEVEL", "+LEVEL=", &level_val, 800) != ESP_OK ||
        level_val != mod->level)
    {
        ESP_LOGE(TAG, "%s: LEVEL=%d, expect %d", mod->name, level_val, mod->level);
        ok = false;
    }
    vTaskDelay(pdMS_TO_TICKS(50));

    if (send_at_query_cmd(mod, "AT+CHANNEL", "+CHANNEL=", &chan_val, 800) != ESP_OK ||
        chan_val != mod->channel)
    {
        ESP_LOGE(TAG, "%s: CHANNEL=%02X, expect %02X", mod->name, chan_val, mod->channel);
        ok = false;
    }

    // 退出AT模式回传输态
    if (send_plus_plus_plus(mod, &new_state) == ESP_OK &&
        new_state == AT_STATE_TRANSMIT)
    {
        wait_module_reset_complete(mod, 3000);
    }
    else
    {
        ESP_LOGW(TAG, "%s: Exit failed in verify, force reset", mod->name);
        send_at_reset_no_wait(mod);
    }

    ESP_LOGI(TAG, "%s: Verify %s (MODE=%d LEVEL=%d CH=%02X)",
             mod->name, ok ? "PASS" : "FAIL", mode_val, level_val, chan_val);
    return ok;
}

bool configure_and_verify(lora_module_t *mod)
{
    for (int attempt = 1; attempt <= CONFIG_VERIFY_ATTEMPTS; attempt++)
    {
        if (attempt > 1)
        {
            ESP_LOGW(TAG, "%s: Reconfig attempt %d/%d", mod->name,
                     attempt, CONFIG_VERIFY_ATTEMPTS);
            clear_uart_buffer(mod->uart_num);
            vTaskDelay(pdMS_TO_TICKS(500));
        }

        if (!configure_lora_module(mod))
            continue;

        if (!mod->enable_verify)
        {
            ESP_LOGI(TAG, "%s: Skip verification", mod->name);
            return true;
        }

        if (verify_module_config(mod))
        {
            return true;
        }
    }
    return false;
}

/* ===================== 数据发送 ===================== */

esp_err_t lora_send_bytes(lora_module_t *mod, const uint8_t *data, size_t len)
{
    if (mod == NULL || data == NULL || len == 0)
    {
        return ESP_ERR_INVALID_ARG;
    }

    // LoRa传输头开销:定点=3字节(地址2+信道1),广播=1字节,透明=0
    size_t head_len = (mod->mode == 1) ? 3 : (mod->mode == 2) ? 1
                                                              : 0;
    if (len + head_len > LORA_UART_BUF_SIZE)
    {
        return ESP_ERR_INVALID_SIZE;
    }

    uint8_t tx_packet[LORA_UART_BUF_SIZE];
    size_t idx = 0;
    if (mod->mode == 1)
    {
        tx_packet[idx++] = mod->target_addr_h;
        tx_packet[idx++] = mod->target_addr_l;
        tx_packet[idx++] = mod->target_channel;
    }
    else if (mod->mode == 2)
    {
        tx_packet[idx++] = mod->target_channel;
    }
    memcpy(&tx_packet[idx], data, len);

    if (!wait_lora_idle(mod, AUX_WAIT_TIMEOUT))
    {
        ESP_LOGE(TAG, "%s: AUX busy", mod->name);
        return ESP_ERR_TIMEOUT;
    }

    uart_write_bytes(mod->uart_num, tx_packet, idx + len);
    ESP_LOGD(TAG, "%s: Sent %d bytes (payload %d + head %d)",
             mod->name, (int)(idx + len), (int)len, (int)head_len);
    return ESP_OK;
}

esp_err_t lora_send_data(lora_module_t *mod, const char *data)
{
    if (data == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }
    return lora_send_bytes(mod, (const uint8_t *)data, strlen(data));
}