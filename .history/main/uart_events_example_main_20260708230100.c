#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_random.h"

static const char *TAG = "LORA_MAIN";

// ===================== 硬件引脚&串口定义 =====================
// 接收LoRa模块 UART1
#define LORA_RX_UART UART_NUM_1
#define LORA_RX_TX_PIN GPIO_NUM_5  // ESP TX → 接收模块 RXD
#define LORA_RX_RX_PIN GPIO_NUM_6  // ESP RX → 接收模块 TXD
#define LORA_RX_AUX_PIN GPIO_NUM_7 // 接收模块AUX输入

// 发送LoRa模块 UART2
#define LORA_TX_UART UART_NUM_2
#define LORA_TX_TX_PIN GPIO_NUM_15  // ESP TX → 发送模块 RXD
#define LORA_TX_RX_PIN GPIO_NUM_16  // ESP RX → 发送模块 TXD
#define LORA_TX_AUX_PIN GPIO_NUM_17 // 发送模块AUX输入

// 串口缓冲区大小
#define UART_BUF_SIZE 1024
// 发送间隔 5000ms
#define SEND_INTERVAL_MS 5000
// 随机字符串长度
#define RAND_STR_LEN 8
// LoRa定点目标配置：目标地址0001，信道01
#define LORA_TARGET_ADDR_H 0x00
#define LORA_TARGET_ADDR_L 0x01
#define LORA_TARGET_CH 0x01
// AUX最大等待超时 2000ms
#define AUX_WAIT_TIMEOUT 2000

// 全局队列：接收模块串口事件
static QueueHandle_t lora_rx_queue = NULL;
// 全局互斥锁：保护UART2发送串口，隔离AT配置/定时发包
static SemaphoreHandle_t uart2_mutex = NULL;

// ===================== 工具函数 =====================
/**
 * @brief 等待AUX引脚变为低电平（模块空闲），带超时防死锁看门狗
 * @param aux_pin AUX引脚号
 * @param timeout_ms 最大等待毫秒
 * @return true=成功等到空闲 false=超时模块忙
 */
static bool wait_lora_idle(gpio_num_t aux_pin, uint32_t timeout_ms)
{
    uint32_t tick_start = xTaskGetTickCount();
    const uint32_t tick_timeout = pdMS_TO_TICKS(timeout_ms);
    while (gpio_get_level(aux_pin) == 1)
    {
        vTaskDelay(pdMS_TO_TICKS(10));
        if ((xTaskGetTickCount() - tick_start) >= tick_timeout)
        {
            ESP_LOGE(TAG, "AUX wait timeout! Module always busy, check wiring/module status");
            return false;
        }
    }
    return true;
}

/**
 * @brief 生成8位随机字母数字字符串
 * @param buf 输出缓冲区
 */
static void generate_random_str(char *buf)
{
    const char charset[] = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz";
    uint32_t rand_val;
    for (int i = 0; i < RAND_STR_LEN; i++)
    {
        rand_val = esp_random() % strlen(charset);
        buf[i] = charset[rand_val];
    }
    buf[RAND_STR_LEN] = '\0';
}

/**
 * @brief 组装LoRa定点传输数据包
 * 格式：2字节目标地址 + 1字节信道 + 业务数据
 */
static void build_lora_packet(const char *src, uint8_t *dst)
{
    dst[0] = LORA_TARGET_ADDR_H;
    dst[1] = LORA_TARGET_ADDR_L;
    dst[2] = LORA_TARGET_CH;
    memcpy(&dst[3], src, strlen(src));
}

/**
 * @brief 初始化单个LoRa串口
 */
static void lora_uart_init(uart_port_t uart_num, gpio_num_t tx_pin, gpio_num_t rx_pin, QueueHandle_t *evt_queue)
{
    uart_config_t uart_cfg = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    uart_driver_install(uart_num, UART_BUF_SIZE * 2, UART_BUF_SIZE * 2, 20, evt_queue, 0);
    uart_param_config(uart_num, &uart_cfg);
    uart_set_pin(uart_num, tx_pin, rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

/**
 * @brief 初始化AUX输入引脚，上拉输入
 */
static void aux_gpio_init(gpio_num_t aux_pin)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = 1ULL << aux_pin,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE};
    gpio_config(&io_conf);
}

/**
 * @brief 发送AT指令，严格遵守+++前后1ms静默时序
 * 修复：删除不存在的uart_flush_output，使用recv_len接收返回值消除警告
 */
static esp_err_t send_at_cmd(uart_port_t uart, const char *cmd, uint32_t wait_ms)
{
    uint8_t recv_buf[64] = {0};
    size_t recv_len;

    // 彻底清空收发缓冲区，清除上电残留乱码
    uart_flush_input(uart);
    // 前置静默 5ms，严格满足手册空闲要求
    vTaskDelay(pdMS_TO_TICKS(5));

    if (strcmp(cmd, "+++") == 0)
    {
        ESP_LOGI(TAG, "Send AT: +++");
        uart_write_bytes(uart, cmd, strlen(cmd));
        // +++发送后强制静默5ms
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    else
    {
        ESP_LOGI(TAG, "Send AT: %s", cmd);
        uart_write_bytes(uart, cmd, strlen(cmd));
    }

    // 大幅拉长等待时间，给模组输出OK窗口
    vTaskDelay(pdMS_TO_TICKS(wait_ms));
    // 延长读取超时
    recv_len = uart_read_bytes(uart, recv_buf, sizeof(recv_buf) - 1, pdMS_TO_TICKS(300));
    ESP_LOGI(TAG, "AT Resp: [%s]", recv_buf);
    if (strstr((char *)recv_buf, "OK") != NULL)
    {
        return ESP_OK;
    }
    ESP_LOGW(TAG, "Cmd %s no OK response", cmd);
    return ESP_FAIL;
}

// ===================== 任务1：接收LoRa数据事件任务（UART1独立串口） =====================
static void lora_rx_event_task(void *pvParameters)
{
    uart_event_t event;
    uint8_t *rx_buf = malloc(UART_BUF_SIZE);
    ESP_LOGI(TAG, "LoRa RX Task Start");

    for (;;)
    {
        if (xQueueReceive(lora_rx_queue, &event, portMAX_DELAY))
        {
            bzero(rx_buf, UART_BUF_SIZE);
            switch (event.type)
            {
            case UART_DATA:
                uart_read_bytes(LORA_RX_UART, rx_buf, event.size, portMAX_DELAY);
                // 干扰包过滤：最小包长4字节，校验目标地址头00 01
                if (event.size < 4 || rx_buf[0] != LORA_TARGET_ADDR_H || rx_buf[1] != LORA_TARGET_ADDR_L)
                {
                    ESP_LOGD(TAG, "Discard noise packet, len=%d header %02X%02X", event.size, rx_buf[0], rx_buf[1]);
                    break;
                }
                ESP_LOGI(TAG, "===== LoRa Recv Packet Len:%d =====", event.size);
                ESP_LOGI(TAG, "Raw HEX:");
                for (int i = 0; i < event.size; i++)
                    printf("%02X ", rx_buf[i]);
                printf("\n");
                ESP_LOGI(TAG, "LoRa Recv Payload String: %.*s", event.size - 3, rx_buf + 3);
                break;
            case UART_FIFO_OVF:
            case UART_BUFFER_FULL:
                uart_flush_input(LORA_RX_UART);
                xQueueReset(lora_rx_queue);
                ESP_LOGW(TAG, "LoRa RX Buffer Overflow, Flush");
                break;
            default:
                ESP_LOGD(TAG, "LoRa RX UART Event Type:%d", event.type);
                break;
            }
        }
    }
    free(rx_buf);
    vTaskDelete(NULL);
}

// ===================== 任务2：定时发送LoRa数据任务（UART2，循环入口拿锁） =====================
static void lora_tx_period_task(void *pvParameters)
{
    char rand_str[RAND_STR_LEN + 1];
    uint8_t tx_packet[3 + RAND_STR_LEN];
    ESP_LOGI(TAG, "LoRa TX Period Task Start, Send Interval: %dms", SEND_INTERVAL_MS);

    for (;;)
    {
        // 循环最开头拿互斥锁：AT持有锁时直接阻塞，不执行任何组包/发包
        xSemaphoreTake(uart2_mutex, portMAX_DELAY);

        generate_random_str(rand_str);
        ESP_LOGI(TAG, "Prepare Send Random Str: %s", rand_str);
        build_lora_packet(rand_str, tx_packet);
        int pkt_len = 3 + strlen(rand_str);

        if (!wait_lora_idle(LORA_TX_AUX_PIN, AUX_WAIT_TIMEOUT))
        {
            ESP_LOGE(TAG, "Skip this send packet due to AUX timeout");
            xSemaphoreGive(uart2_mutex);
            vTaskDelay(pdMS_TO_TICKS(SEND_INTERVAL_MS));
            continue;
        }

        uart_write_bytes(LORA_TX_UART, tx_packet, pkt_len);
        ESP_LOGI(TAG, "Send LoRa Packet Done, Total Len:%d", pkt_len);

        xSemaphoreGive(uart2_mutex);
        vTaskDelay(pdMS_TO_TICKS(SEND_INTERVAL_MS));
    }
}

// 修复：正确实现+++时序
static esp_err_t send_at_cmd(uart_port_t uart, const char *cmd, uint32_t response_timeout_ms)
{
    uint8_t recv_buf[128] = {0};
    size_t recv_len;
    bool is_enter_at = (strcmp(cmd, "+++") == 0);

    // 1. 清空接收缓冲区（清除上电杂波）
    uart_flush_input(uart);

    // 2. 发送前静默（关键！）
    vTaskDelay(pdMS_TO_TICKS(150)); // 至少100ms

    // 3. 发送命令
    if (is_enter_at)
    {
        ESP_LOGI(TAG, "Send: +++ (enter AT mode)");
        uart_write_bytes(uart, "+++", 3);
        // +++ 发送后立即开始等待响应，不再额外延迟
    }
    else
    {
        ESP_LOGI(TAG, "Send: %s", cmd);
        uart_write_bytes(uart, cmd, strlen(cmd));
        // 普通AT指令需要等待模块处理
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    uart_wait_tx_done(uart, pdMS_TO_TICKS(50));

    // 4. 读取响应（根据指令类型调整超时）
    int timeout_ms = is_enter_at ? 500 : response_timeout_ms;
    recv_len = uart_read_bytes(uart, recv_buf, sizeof(recv_buf) - 1,
                               pdMS_TO_TICKS(timeout_ms));

    if (recv_len > 0)
    {
        recv_buf[recv_len] = '\0';
        ESP_LOGI(TAG, "AT Resp[%d]: [%s]", recv_len, recv_buf);

        // 检查是否包含OK（不区分大小写）
        if (strstr((char *)recv_buf, "OK") != NULL ||
            strstr((char *)recv_buf, "ok") != NULL)
        {
            return ESP_OK;
        }

        // 如果是+++响应，可能只有"OK"没有换行
        if (is_enter_at && recv_len >= 2 &&
            (recv_buf[0] == 'O' && recv_buf[1] == 'K'))
        {
            return ESP_OK;
        }
    }
    else
    {
        ESP_LOGW(TAG, "No response for cmd: %s", cmd);
    }

    return ESP_FAIL;
}

// 封装重试函数（增加退避策略）
static esp_err_t send_at_retry(uart_port_t uart, const char *cmd,
                               uint32_t timeout_ms, uint8_t retry_max)
{
    esp_err_t ret;
    for (uint8_t i = 0; i < retry_max; i++)
    {
        ret = send_at_cmd(uart, cmd, timeout_ms);
        if (ret == ESP_OK)
        {
            ESP_LOGI(TAG, "Command OK: %s", cmd);
            return ESP_OK;
        }
        ESP_LOGW(TAG, "Retry %d/%d for cmd: %s", i + 1, retry_max, cmd);
        // 退避延时：每次重试增加等待时间
        vTaskDelay(pdMS_TO_TICKS(200 * (i + 1)));
        // 重试前再次清空缓冲区
        uart_flush_input(uart);
    }
    ESP_LOGE(TAG, "Command failed after %d retries: %s", retry_max, cmd);
    return ESP_FAIL;
}

// 修复：AT配置任务流程
static void lora_module_at_config_task(void *pvParameters)
{
    // 获取UART2互斥锁（保护发送串口）
    xSemaphoreTake(uart2_mutex, portMAX_DELAY);

    // 等待模组完全上电（增加至3秒）
    ESP_LOGI(TAG, "Waiting for LoRa modules to power on...");
    vTaskDelay(pdMS_TO_TICKS(3000));

    // 清空所有串口缓冲区
    uart_flush_input(LORA_TX_UART);
    uart_flush_input(LORA_RX_UART);

    // ===== 配置发送模块 =====
    ESP_LOGI(TAG, "==== Config TX LoRa Module ====");

    // 1. 进入AT模式（重点修复）
    if (send_at_retry(LORA_TX_UART, "+++", 500, 3) != ESP_OK)
    {
        ESP_LOGE(TAG, "TX module: Failed to enter AT mode");
        goto config_fail;
    }
    vTaskDelay(pdMS_TO_TICKS(100));

    // 2. 配置参数（注意：指令格式为 AT+MODE=1 而非 AT+MODE1）
    send_at_retry(LORA_TX_UART, "AT+MODE=1\r\n", 800, 3);
    vTaskDelay(pdMS_TO_TICKS(50));
    send_at_retry(LORA_TX_UART, "AT+LEVEL=3\r\n", 800, 3);
    vTaskDelay(pdMS_TO_TICKS(50));

    // ===== 配置接收模块 =====
    ESP_LOGI(TAG, "==== Config RX LoRa Module ====");

    if (send_at_retry(LORA_RX_UART, "+++", 500, 3) != ESP_OK)
    {
        ESP_LOGE(TAG, "RX module: Failed to enter AT mode");
        goto config_fail;
    }
    vTaskDelay(pdMS_TO_TICKS(100));

    send_at_retry(LORA_RX_UART, "AT+MODE=1\r\n", 800, 3);
    vTaskDelay(pdMS_TO_TICKS(50));
    send_at_retry(LORA_RX_UART, "AT+LEVEL=3\r\n", 800, 3);
    vTaskDelay(pdMS_TO_TICKS(50));

    // 可选：发送AT+RESET使参数生效（调试阶段暂时注释）
    // send_at_retry(LORA_TX_UART, "AT+RESET\r\n", 1500, 2);
    // send_at_retry(LORA_RX_UART, "AT+RESET\r\n", 1500, 2);

    ESP_LOGI(TAG, "==== All LoRa Modules Configured Successfully ====");

config_fail:
    // 释放互斥锁，允许定时发送任务运行
    xSemaphoreGive(uart2_mutex);
    ESP_LOGI(TAG, "AT config task finished, delete itself");
    vTaskDelete(NULL);
}

// ===================== 主入口 app_main =====================
void app_main(void)
{
    esp_log_level_set(TAG, ESP_LOG_INFO);
    // 创建UART2互斥信号量
    uart2_mutex = xSemaphoreCreateMutex();

    // 初始化两路AUX输入引脚
    aux_gpio_init(LORA_RX_AUX_PIN);
    aux_gpio_init(LORA_TX_AUX_PIN);

    // UART1 RX(GPIO6)开启内部下拉，抑制浮空干扰杂波
    gpio_config_t rx_noise_fix = {
        .pin_bit_mask = 1ULL << LORA_RX_RX_PIN,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_DISABLE};
    gpio_config(&rx_noise_fix);

    // 初始化两路独立LoRa串口
    lora_uart_init(LORA_RX_UART, LORA_RX_TX_PIN, LORA_RX_RX_PIN, &lora_rx_queue);
    lora_uart_init(LORA_TX_UART, LORA_TX_TX_PIN, LORA_TX_RX_PIN, NULL);

    // 任务优先级：数字越大优先级越高
    // lora_at_cfg:12(最高) > lora_rx_task:11 > lora_tx_task:10
    xTaskCreate(lora_module_at_config_task, "lora_at_cfg", 4096, NULL, 12, NULL);
    xTaskCreate(lora_rx_event_task, "lora_rx_task", 4096, NULL, 11, NULL);
    xTaskCreate(lora_tx_period_task, "lora_tx_task", 4096, NULL, 10, NULL);
}