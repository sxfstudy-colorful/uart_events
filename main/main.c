#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_random.h"
#include "lora_uart.h"

static const char *TAG = "LORA_MAIN";

#define AUX_CONNECTED 0

#define LORA_RX_UART UART_NUM_1
#define LORA_RX_TX_PIN GPIO_NUM_5
#define LORA_RX_RX_PIN GPIO_NUM_6
#define LORA_RX_AUX_PIN GPIO_NUM_7

#define LORA_TX_UART UART_NUM_2
#define LORA_TX_TX_PIN GPIO_NUM_15
#define LORA_TX_RX_PIN GPIO_NUM_16
#define LORA_TX_AUX_PIN GPIO_NUM_17

#define SEND_INTERVAL_MS 1000
#define RAND_STR_LEN 8

#define ENABLE_CONTROLLER 1
#define ENABLE_RECEIVER 1

static QueueHandle_t lora_rx_queue = NULL;
static SemaphoreHandle_t uart_mutex = NULL;
static EventGroupHandle_t sys_evt_group = NULL;
#define EVT_TX_CONFIG_DONE BIT0
#define EVT_RX_CONFIG_DONE BIT1

static lora_module_t g_rx_module = {
    .name = "RX-Mod",
    .uart_num = LORA_RX_UART,
    .tx_pin = LORA_RX_TX_PIN,
    .rx_pin = LORA_RX_RX_PIN,
    .aux_pin = LORA_RX_AUX_PIN,
    .evt_queue = &lora_rx_queue,
    .is_rx = true,
    .mode = 1,
    .level = 3,
    .enable_verify = 0,
    .addr_h = 0x00,
    .addr_l = 0x01,
    .channel = 0x01,
    .target_addr_h = 0x00,
    .target_addr_l = 0x02,
    .target_channel = 0x01,
    .rx_callback = NULL,
};

static lora_module_t g_tx_module = {
    .name = "TX-Mod",
    .uart_num = LORA_TX_UART,
    .tx_pin = LORA_TX_TX_PIN,
    .rx_pin = LORA_TX_RX_PIN,
    .aux_pin = LORA_TX_AUX_PIN,
    .evt_queue = NULL,
    .is_rx = false,
    .mode = 1,
    .level = 3,
    .enable_verify = 0,
    .addr_h = 0x00,
    .addr_l = 0x02,
    .channel = 0x02,
    .target_addr_h = 0x00,
    .target_addr_l = 0x01,
    .target_channel = 0x01,
    .rx_callback = NULL,
};

static void generate_random_str(char *buf)
{
    const char charset[] = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz";
    for (int i = 0; i < RAND_STR_LEN; i++)
    {
        buf[i] = charset[esp_random() % (uint32_t)strlen(charset)];
    }
    buf[RAND_STR_LEN] = '\0';
}

static void lora_rx_data_callback(const lora_module_t *mod, const uint8_t *data, size_t len)
{
    ESP_LOGI(TAG, "%s: [Callback] Received %d bytes", mod->name, len);
    if (len > 0)
    {
        uint8_t str_buf[LORA_UART_BUF_SIZE];
        memcpy(str_buf, data, len);
        str_buf[len] = '\0';
        ESP_LOGI(TAG, "%s: [Callback] Data: %s", mod->name, (char *)str_buf);
    }
}

static void lora_rx_event_task(void *pvParameters)
{
    const lora_module_t *mod = (const lora_module_t *)pvParameters;
    uart_event_t event;
    uint8_t *rx_buf = (uint8_t *)malloc(LORA_UART_BUF_SIZE);
    if (rx_buf == NULL)
    {
        ESP_LOGE(TAG, "Failed to allocate RX buffer");
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "%s: RX Task waiting for config done...", mod->name);
    xEventGroupWaitBits(sys_evt_group, EVT_RX_CONFIG_DONE, pdFALSE, pdTRUE, portMAX_DELAY);

    uart_flush_input(mod->uart_num);
    xQueueReset(*mod->evt_queue);
    ESP_LOGI(TAG, "%s: RX Task Start", mod->name);

    for (;;)
    {
        if (xQueueReceive(*mod->evt_queue, &event, portMAX_DELAY))
        {
            memset(rx_buf, 0, LORA_UART_BUF_SIZE);
            switch (event.type)
            {
            case UART_DATA:
                uart_read_bytes(mod->uart_num, rx_buf, event.size, portMAX_DELAY);
                ESP_LOGI(TAG, "===== %s Recv Len:%d =====", mod->name, event.size);
                ESP_LOGI(TAG, "Raw HEX:");
                for (int i = 0; i < event.size; i++)
                    printf("%02X ", rx_buf[i]);
                printf("\n");
                if (event.size < LORA_UART_BUF_SIZE)
                {
                    rx_buf[event.size] = '\0';
                    ESP_LOGI(TAG, "%s: Raw String: %s", mod->name, (char *)rx_buf);
                    if (mod->mode == 0)
                    {
                        ESP_LOGI(TAG, "%s: Payload: %s", mod->name, (char *)rx_buf);
                        if (mod->rx_callback)
                        {
                            mod->rx_callback(mod, rx_buf, event.size);
                        }
                    }
                    else if (event.size >= 4 &&
                             rx_buf[0] == mod->addr_h &&
                             rx_buf[1] == mod->addr_l)
                    {
                        ESP_LOGI(TAG, "%s: Payload: %s", mod->name, (char *)&rx_buf[3]);
                        if (mod->rx_callback)
                        {
                            mod->rx_callback(mod, &rx_buf[3], event.size - 3);
                        }
                    }
                }
                break;
            case UART_FIFO_OVF:
            case UART_BUFFER_FULL:
                uart_flush_input(mod->uart_num);
                xQueueReset(*mod->evt_queue);
                ESP_LOGW(TAG, "%s: Buffer Overflow", mod->name);
                break;
            default:
                ESP_LOGD(TAG, "%s: Event type:%d", mod->name, event.type);
                break;
            }
        }
    }
    free(rx_buf);
    vTaskDelete(NULL);
}

static void lora_tx_period_task(void *pvParameters)
{
    const lora_module_t *mod = (const lora_module_t *)pvParameters;
    char rand_str[RAND_STR_LEN + 1];
    uint8_t tx_packet[3 + RAND_STR_LEN];

    ESP_LOGI(TAG, "%s: TX Task waiting for config done...", mod->name);
    xEventGroupWaitBits(sys_evt_group, EVT_TX_CONFIG_DONE, pdFALSE, pdTRUE, portMAX_DELAY);

    ESP_LOGI(TAG, "%s: TX Task Start, Interval: %dms", mod->name, SEND_INTERVAL_MS);

    for (;;)
    {
        xSemaphoreTake(uart_mutex, portMAX_DELAY);

        generate_random_str(rand_str);
        ESP_LOGI(TAG, "%s: Send: %s", mod->name, rand_str);
        build_lora_packet(mod, rand_str, tx_packet);
        int pkt_len = (mod->mode == 0) ? strlen(rand_str) : (3 + strlen(rand_str));

        if (!wait_lora_idle(mod, AUX_WAIT_TIMEOUT))
        {
            ESP_LOGE(TAG, "%s: AUX busy, skip", mod->name);
            xSemaphoreGive(uart_mutex);
            vTaskDelay(pdMS_TO_TICKS(SEND_INTERVAL_MS));
            continue;
        }

        uart_write_bytes(mod->uart_num, tx_packet, pkt_len);
        ESP_LOGI(TAG, "%s: Sent %d bytes", mod->name, pkt_len);

        xSemaphoreGive(uart_mutex);
        vTaskDelay(pdMS_TO_TICKS(50));
        ESP_LOGI(TAG, "%s: Wait for TX complete, allowing RX...", mod->name);
        vTaskDelay(pdMS_TO_TICKS(SEND_INTERVAL_MS - 50));
    }
}

static void lora_module_at_config_task(void *pvParameters)
{
    const lora_module_t *mod = (const lora_module_t *)pvParameters;

    xSemaphoreTake(uart_mutex, portMAX_DELAY);

    ESP_LOGI(TAG, "Waiting for %s to power on...", mod->name);
    vTaskDelay(pdMS_TO_TICKS(3000));

    clear_uart_buffer(mod->uart_num);

    bool ok = configure_and_verify(mod);

    xSemaphoreGive(uart_mutex);
    if (mod->is_rx)
    {
        xEventGroupSetBits(sys_evt_group, EVT_RX_CONFIG_DONE);
    }
    else
    {
        xEventGroupSetBits(sys_evt_group, EVT_TX_CONFIG_DONE);
    }

    ESP_LOGI(TAG, "==== Config Result: %s=%s ====", mod->name, ok ? "PASS" : "FAIL");
    vTaskDelete(NULL);
}

void app_main(void)
{
    esp_log_level_set(TAG, ESP_LOG_INFO);

    ESP_LOGI(TAG, "==== Device Mode: CONTROLLER=%d, RECEIVER=%d ====",
             ENABLE_CONTROLLER, ENABLE_RECEIVER);

    uart_mutex = xSemaphoreCreateMutex();
    sys_evt_group = xEventGroupCreate();
    if (uart_mutex == NULL || sys_evt_group == NULL)
    {
        ESP_LOGE(TAG, "Failed to create sync objects");
        return;
    }

#ifdef ENABLE_CONTROLLER
#if AUX_CONNECTED
    aux_gpio_init(g_tx_module.aux_pin);
#endif
    lora_uart_init(&g_tx_module, NULL);
    xTaskCreate(lora_module_at_config_task, "lora_tx_cfg", 4096, (void *)&g_tx_module, 12, NULL);
#endif

#ifdef ENABLE_RECEIVER
#if AUX_CONNECTED
    aux_gpio_init(g_rx_module.aux_pin);
#endif
    lora_uart_init(&g_rx_module, &lora_rx_queue);
    xTaskCreate(lora_module_at_config_task, "lora_rx_cfg", 4096, (void *)&g_rx_module, 12, NULL);
#endif

#if ENABLE_CONTROLLER

#if ENABLE_RECEIVER
    xEventGroupWaitBits(sys_evt_group, EVT_RX_CONFIG_DONE, pdFALSE, pdTRUE, portMAX_DELAY);
#endif
    xEventGroupWaitBits(sys_evt_group, EVT_TX_CONFIG_DONE, pdFALSE, pdTRUE, portMAX_DELAY);
    xTaskCreate(lora_tx_period_task, "lora_tx_task", 4096, (void *)&g_tx_module, 10, NULL);
#endif

#if ENABLE_RECEIVER
#if ENABLE_CONTROLLER
    xEventGroupWaitBits(sys_evt_group, EVT_TX_CONFIG_DONE, pdFALSE, pdTRUE, portMAX_DELAY);
#endif
    xEventGroupWaitBits(sys_evt_group, EVT_RX_CONFIG_DONE, pdFALSE, pdTRUE, portMAX_DELAY);
    g_rx_module.rx_callback = lora_rx_data_callback;
    xTaskCreate(lora_rx_event_task, "lora_rx_task", 4096, (void *)&g_rx_module, 11, NULL);
#endif
}
