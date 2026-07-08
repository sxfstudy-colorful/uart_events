#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_random.h"

static const char *TAG = "LORA_MAIN";

// ===================== 硬件引脚&串口定义 =====================
// 接收LoRa模块 UART1
#define LORA_RX_UART UART_NUM_1
#define LORA_RX_TX_PIN GPIO_NUM_5  // ESP TX → LoRa RXD
#define LORA_RX_RX_PIN GPIO_NUM_6  // ESP RX → LoRa TXD
#define LORA_RX_AUX_PIN GPIO_NUM_7 // AUX状态输入

// 发送LoRa模块 UART2
#define LORA_TX_UART UART_NUM_2
#define LORA_TX_TX_PIN GPIO_NUM_15  // ESP TX → LoRa RXD
#define LORA_TX_RX_PIN GPIO_NUM_16  // ESP RX → LoRa TXD
#define LORA_TX_AUX_PIN GPIO_NUM_17 // AUX状态输入

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
// 全局互斥锁：保护发送串口，防止AT配置/定时发包冲突
static SemaphoreHandle_t uart2_mutex = NULL;

// ===================== 工具函数 =====================
/**
 * @brief 等待AUX引脚变为低电平（模块空闲），带超时防死锁
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
        .baud_rate = 115200,
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
 * @brief 初始化AUX输入引脚
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
 * @brief 发送AT指令并读取返回应答，修复+++静默间隔
 */
static esp_err_t send_at_cmd(uart_port_t uart, const char *cmd, uint32_t wait_ms)
{
    uint8_t recv_buf[64] = {0};
    size_t recv_len;
    uart_flush_input(uart);

    // 关键修复：进入AT模式的+++需要前后1ms静默
    if (strcmp(cmd, "+++") == 0)
    {
        vTaskDelay(pdMS_TO_TICKS(1));
        ESP_LOGI(TAG, "Send AT: +++");
        uart_write_bytes(uart, cmd, strlen(cmd));
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    else
    {
        ESP_LOGI(TAG, "Send AT: %s", cmd);
        uart_write_bytes(uart, cmd, strlen(cmd));
    }

    vTaskDelay(pdMS_TO_TICKS(wait_ms));
    recv_len = uart_read_bytes(uart, recv_buf, sizeof(recv_buf) - 1, pdMS_TO_TICKS(100));
    ESP_LOGI(TAG, "AT Resp: [%s]", recv_buf);
    if (strstr((char *)recv_buf, "OK") != NULL)
        return ESP_OK;
    return ESP_FAIL;
}

// ===================== 任务1：接收LoRa数据任务 =====================
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
                // ========== 干扰包过滤逻辑 ==========
                // 最小合法包长度：3字节头 + 1字节数据 = 4
                if (event.size < 4)
                {
                    ESP_LOGD(TAG, "Discard noise packet, len=%d", event.size);
                    break;
                }
                // 校验目标地址头
                if (rx_buf[0] != LORA_TARGET_ADDR_H || rx_buf[1] != LORA_TARGET_ADDR_L)
                {
                    ESP_LOGD(TAG, "Discard noise packet, wrong header %02X %02X", rx_buf[0], rx_buf[1]);
                    break;
                }
                // ===================================
                ESP_LOGI(TAG, "===== LoRa Recv Packet Len:%d =====", event.size);
                ESP_LOGI(TAG, "Raw HEX:");
                for (int i = 0; i < event.size; i++)
                {
                    printf("%02X ", rx_buf[i]);
                }
                printf("\n");
                // 打印业务字符串载荷
                ESP_LOGI(TAG, "Payload String: %.*s", event.size - 3, rx_buf + 3);
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

// ===================== 任务2：定时发送LoRa数据任务 =====================
static void lora_tx_period_task(void *pvParameters)
{
    char rand_str[RAND_STR_LEN + 1];
    uint8_t tx_packet[3 + RAND_STR_LEN];
    ESP_LOGI(TAG, "LoRa TX Period Task Start, Send Interval: %dms", SEND_INTERVAL_MS);

    for (;;)
    {
        generate_random_str(rand_str);
        ESP_LOGI(TAG, "Prepare Send Random Str: %s", rand_str);
        build_lora_packet(rand_str, tx_packet);
        int pkt_len = 3 + strlen(rand_str);

        // 加互斥锁，防止和AT配置抢占UART2
        xSemaphoreTake(uart2_mutex, portMAX_DELAY);

        // 等待模块空闲，带超时保护
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

// ===================== 任务3：模块AT初始化配置（最高优先级，先执行） =====================
static void lora_module_at_config_task(void *pvParameters)
{
    vTaskDelay(pdMS_TO_TICKS(1500)); // 等待模块上电稳定

    // 占用串口互斥锁，独占UART2完成全部配置
    xSemaphoreTake(uart2_mutex, portMAX_DELAY);

    // 配置发送模块
    ESP_LOGI(TAG, "==== Config TX LoRa Module ====");
    send_at_cmd(LORA_TX_UART, "+++", 300);
    send_at_cmd(LORA_TX_UART, "AT+MODE1\r\n", 500);
    send_at_cmd(LORA_TX_UART, "AT+LEVEL3\r\n", 500);
    send_at_cmd(LORA_TX_UART, "AT+RESET\r\n", 800);

    vTaskDelay(pdMS_TO_TICKS(1200)); // 等待模块重启

    // 配置接收模块
    ESP_LOGI(TAG, "==== Config RX LoRa Module ====");
    send_at_cmd(LORA_RX_UART, "+++", 300);
    send_at_cmd(LORA_RX_UART, "AT+MODE1\r\n", 500);
    send_at_cmd(LORA_RX_UART, "AT+LEVEL3\r\n", 500);
    send_at_cmd(LORA_RX_UART, "AT+RESET\r\n", 800);

    xSemaphoreGive(uart2_mutex);
    ESP_LOGI(TAG, "All LoRa Module Config Finished, Enter Transmit Mode");
    vTaskDelete(NULL);
}

// ===================== 主入口 =====================
void app_main(void)
{
    esp_log_level_set(TAG, ESP_LOG_INFO);

    // 创建串口互斥锁，隔离AT配置和定时发包
    uart2_mutex = xSemaphoreCreateMutex();

    // 1. 初始化AUX GPIO
    aux_gpio_init(LORA_RX_AUX_PIN);
    aux_gpio_init(LORA_TX_AUX_PIN);

    // 2. UART1 RX(GPIO6)开启内部下拉，抑制浮空干扰
    gpio_config_t rx_noise_fix = {
        .pin_bit_mask = 1ULL << LORA_RX_RX_PIN,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_DISABLE};
    gpio_config(&rx_noise_fix);

    // 3. 初始化两路LoRa串口
    lora_uart_init(LORA_RX_UART, LORA_RX_TX_PIN, LORA_RX_RX_PIN, &lora_rx_queue);
    lora_uart_init(LORA_TX_UART, LORA_TX_TX_PIN, LORA_TX_RX_PIN, NULL);

    // 4. 调整任务优先级：AT配置任务优先级最高，优先执行完成
    // 优先级数字越大优先级越高
    xTaskCreate(lora_module_at_config_task, "lora_at_cfg", 2048, NULL, 12, NULL);
    xTaskCreate(lora_rx_event_task, "lora_rx_task", 4096, NULL, 11, NULL);
    xTaskCreate(lora_tx_period_task, "lora_tx_task", 2048, NULL, 10, NULL);
}