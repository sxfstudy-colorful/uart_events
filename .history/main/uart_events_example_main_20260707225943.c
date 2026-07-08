#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>
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
// LoRa定点目标配置（根据手册示例：目标地址0001，信道01）
#define LORA_TARGET_HEAD "00 01 01 "

// 全局队列：接收模块串口事件
static QueueHandle_t lora_rx_queue = NULL;

// ===================== 工具函数 =====================
/**
 * @brief 等待AUX引脚变为低电平（模块空闲）
 * @param aux_pin AUX引脚号
 */
static void wait_lora_idle(gpio_num_t aux_pin)
{
    while (gpio_get_level(aux_pin) == 1)
    {
        vTaskDelay(pdMS_TO_TICKS(1));
    }
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
 * @brief 字符串转HEX字节流（定点传输前缀+数据）
 * @param src 原始ASCII字符串
 * @param dst 输出十六进制字节数组
 * @param head_len 定点头部长度 3字节(00 01 01)
 */
static void str_to_lora_hex_packet(const char *src, uint8_t *dst, int head_len)
{
    // 定点头部：目标地址0001(2byte) + 信道01(1byte)
    dst[0] = 0x00;
    dst[1] = 0x01;
    dst[2] = 0x01;
    // 填充数据
    for (int i = 0; i < strlen(src); i++)
    {
        dst[head_len + i] = src[i];
    }
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
                // 读取串口收到的定点数据包
                uart_read_bytes(LORA_RX_UART, rx_buf, event.size, portMAX_DELAY);
                ESP_LOGI(TAG, "===== LoRa Recv Packet Len:%d =====", event.size);
                // 定点协议前3字节是地址+信道，有效数据从第3位开始
                ESP_LOGI(TAG, "Raw HEX:");
                for (int i = 0; i < event.size; i++)
                {
                    printf("%02X ", rx_buf[i]);
                }
                printf("\n");
                // 打印业务字符串数据
                if (event.size > 3)
                {
                    ESP_LOGI(TAG, "Payload String: %.*s", event.size - 3, rx_buf + 3);
                }
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
    uint8_t tx_packet[3 + RAND_STR_LEN]; // 3字节头部 + 8字节数据
    ESP_LOGI(TAG, "LoRa TX Period Task Start, Send Interval: %dms", SEND_INTERVAL_MS);

    for (;;)
    {
        // 1. 生成8位随机字符串
        generate_random_str(rand_str);
        ESP_LOGI(TAG, "Prepare Send Random Str: %s", rand_str);

        // 2. 组装定点传输数据包（头部00 01 01 + 随机字符串）
        str_to_lora_hex_packet(rand_str, tx_packet, 3);
        int pkt_len = 3 + strlen(rand_str);

        // 3. 等待发送模块AUX空闲
        wait_lora_idle(LORA_TX_AUX_PIN);

        // 4. 串口发送定点数据包
        uart_write_bytes(LORA_TX_UART, tx_packet, pkt_len);
        ESP_LOGI(TAG, "Send LoRa Packet Done, Total Len:%d", pkt_len);

        // 5. 等待5秒再发下一包
        vTaskDelay(pdMS_TO_TICKS(SEND_INTERVAL_MS));
    }
}

// ===================== 任务3：模块AT初始化配置（仅上电执行一次） =====================
static void lora_module_at_config_task(void *pvParameters)
{
    vTaskDelay(pdMS_TO_TICKS(1000)); // 等待模块上电稳定
    const char *at_cmds[] = {
        "+++",           // 进入AT模式
        "AT+MODE1\r\n",  // 定点传输模式
        "AT+LEVEL3\r\n", // 速率等级3（收发模块必须一致）
        "AT+RESET\r\n"   // 重启生效
    };
    int cmd_cnt = sizeof(at_cmds) / sizeof(char *);

    // 配置发送模块
    ESP_LOGI(TAG, "Config TX LoRa Module AT Commands");
    for (int i = 0; i < cmd_cnt; i++)
    {
        uart_write_bytes(LORA_TX_UART, at_cmds[i], strlen(at_cmds[i]));
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    vTaskDelay(pdMS_TO_TICKS(1000));

    // 配置接收模块
    ESP_LOGI(TAG, "Config RX LoRa Module AT Commands");
    for (int i = 0; i < cmd_cnt; i++)
    {
        uart_write_bytes(LORA_RX_UART, at_cmds[i], strlen(at_cmds[i]));
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    ESP_LOGI(TAG, "All LoRa Module Config Finished, Enter Transmit Mode");
    vTaskDelete(NULL);
}

// ===================== 主入口 =====================
void app_main(void)
{
    esp_log_level_set(TAG, ESP_LOG_INFO);

    // 1. 初始化AUX GPIO
    aux_gpio_init(LORA_RX_AUX_PIN);
    aux_gpio_init(LORA_TX_AUX_PIN);

    // 2. 初始化两路LoRa串口
    lora_uart_init(LORA_RX_UART, LORA_RX_TX_PIN, LORA_RX_RX_PIN, &lora_rx_queue);
    lora_uart_init(LORA_TX_UART, LORA_TX_TX_PIN, LORA_TX_RX_PIN, NULL);

    // 3. 创建任务
    // AT配置一次性任务
    xTaskCreate(lora_module_at_config_task, "lora_at_cfg", 2048, NULL, 10, NULL);
    // 接收串口事件处理任务
    xTaskCreate(lora_rx_event_task, "lora_rx_task", 4096, NULL, 11, NULL);
    // 定时发送任务
    xTaskCreate(lora_tx_period_task, "lora_tx_task", 2048, NULL, 10, NULL);
}