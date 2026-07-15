#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "board_config.h"
#include "lora_uart.h"
#include "comm_frame.h"
#include "cmds.h"
#include "joystick.h"

static const char *TAG = "LORA_MAIN";

/*
 * ============================================================
 *  组装层:按 board_config.h 中的 DEVICE_ROLE 拼装任务。
 *
 *  协议栈(自上而下):
 *    cmds       应用协议(type+msg_id+TLV字段,复用戒指项目引擎)
 *    comm_frame 链路组帧(SOF+LEN+CRC8)
 *    lora_uart  LoRa传输(定点地址+信道头)
 *
 *  【每个模组都有独立的收+发双向路径】
 *  模组UART是全双工(TXD/RXD独立),半双工的只是无线电层。
 *  每个模组配一个事件队列+组帧解析器+通用接收任务,由此:
 *    - 接收器收到摇杆包 → 回ACK(ENABLE_LINK_ACK=1)
 *    - 发送器收到ACK → 本地日志自证送达,分离部署时
 *      无需盯接收器的串口即可检验链路
 * ============================================================
 */

/* ---------- 同步对象 ---------- */
static EventGroupHandle_t sys_evt_group = NULL;
#define EVT_TX_CONFIG_DONE BIT0
#define EVT_RX_CONFIG_DONE BIT1

/* 本设备需要等齐哪些配置完成位(按角色自动推导) */
#define EVT_ALL_CONFIG_DONE \
    ((HAS_TX_MODULE ? EVT_TX_CONFIG_DONE : 0) | (HAS_RX_MODULE ? EVT_RX_CONFIG_DONE : 0))

/* ---------- 每模组接收上下文:模组 + 专属组帧解析器 ---------- */
typedef struct
{
    lora_module_t *mod;
    frame_parser_t parser;
} lora_rx_ctx_t;

/* ===================== 模块描述符 ===================== */

#if HAS_RX_MODULE
static QueueHandle_t lora_rx_queue = NULL;

static lora_module_t g_rx_module = {
    .name = "RX-Mod",
    .uart_num = LORA_RX_UART,
    .tx_pin = LORA_RX_TX_PIN,
    .rx_pin = LORA_RX_RX_PIN,
    .aux_pin = LORA_RX_AUX_PIN,
    .evt_queue = &lora_rx_queue,
    .mode = LORA_NET_MODE,
    .level = LORA_NET_LEVEL,
    .enable_verify = false,
    // 本模组身份 = 接收器
    .addr_h = RECEIVER_ADDR_H,
    .addr_l = RECEIVER_ADDR_L,
    .channel = RECEIVER_CHANNEL,
    // 回传目标 = 遥控器(ACK走这条路)
    .target_addr_h = CONTROLLER_ADDR_H,
    .target_addr_l = CONTROLLER_ADDR_L,
    .target_channel = CONTROLLER_CHANNEL,
    .rx_callback = NULL,
};
static lora_rx_ctx_t s_rx_mod_ctx = {.mod = &g_rx_module};
#endif

#if HAS_TX_MODULE
static QueueHandle_t lora_tx_queue = NULL; // TX模组也要收(ACK回传)

static lora_module_t g_tx_module = {
    .name = "TX-Mod",
    .uart_num = LORA_TX_UART,
    .tx_pin = LORA_TX_TX_PIN,
    .rx_pin = LORA_TX_RX_PIN,
    .aux_pin = LORA_TX_AUX_PIN,
    .evt_queue = &lora_tx_queue,
    .mode = LORA_NET_MODE,
    .level = LORA_NET_LEVEL,
    .enable_verify = false,
    // 本模组身份 = 遥控器
    .addr_h = CONTROLLER_ADDR_H,
    .addr_l = CONTROLLER_ADDR_L,
    .channel = CONTROLLER_CHANNEL,
    // 发送目标 = 接收器
    .target_addr_h = RECEIVER_ADDR_H,
    .target_addr_l = RECEIVER_ADDR_L,
    .target_channel = RECEIVER_CHANNEL,
    .rx_callback = NULL,
};
static lora_rx_ctx_t s_tx_mod_ctx = {.mod = &g_tx_module};
#endif

/* ===================== 消息处理 ===================== */

static uint32_t s_last_rx_msg_id = 0;           // 接收侧:丢包检测
static volatile uint32_t s_last_ack_id = 0;     // 发送侧:最近被ACK的msg_id
static volatile uint32_t s_ack_miss_streak = 0; // 发送侧:连续未ACK计数(链路健康)

/* ---------- 接收侧链路监督(失联感知与failsafe) ---------- */
static volatile TickType_t s_last_joystick_tick = 0; // 最近一次收到摇杆包的时刻
static volatile bool s_link_up = false;              // 接收侧视角的链路状态

/**
 * @brief 失联保护:执行机构进入安全状态
 * 【安全刚需】遥控器没电/超出范围时,若保持最后的摇杆值,
 * 电机会一直转——超时必须归零输出。
 */
static void failsafe_activate(void)
{
    ESP_LOGE(TAG, "!!! FAILSAFE: link lost, outputs neutralized !!!");
    // TODO: 执行机构归零,如 motor_set_speed(0, 0); servo_center();
}

/**
 * @brief 链路恢复:退出保护状态
 */
static void failsafe_release(void)
{
    ESP_LOGW(TAG, "=== LINK RESTORED, resume control ===");
    // TODO: 如需恢复动作(通常无:等下一包摇杆数据自然驱动即可)
}

/**
 * @brief 发送一个ACK(经指定模组,目标为其target配置)
 */
static void send_link_ack(lora_module_t *mod, uint32_t acked_msg_id)
{
    uint8_t cmd_buf[16];
    uint8_t frame_buf[16 + FRAME_OVERHEAD];
    uint16_t cmd_len = 0;

    if (link_ack_write(cmd_buf, sizeof(cmd_buf), &cmd_len, acked_msg_id) != CMD_SUCCESS)
    {
        return;
    }
    int frame_len = frame_encode(cmd_buf, (uint8_t)cmd_len, frame_buf, sizeof(frame_buf));
    if (frame_len > 0)
    {
        lora_send_bytes(mod, frame_buf, (size_t)frame_len);
        ESP_LOGD(TAG, "%s: ACK[%lu] sent", mod->name, (unsigned long)acked_msg_id);
    }
}

/**
 * @brief 摇杆消息处理(在收到该包的模组上下文中执行)
 * 【实际控制逻辑接入点】:后续在此驱动电机/舵机等执行机构
 */
static void handle_joystick_pkt(lora_module_t *mod, const uint8_t *packet, uint16_t len)
{
    joystick_msg_t js = {0};
    uint32_t msg_id = 0;

    cmd_status_t st = joystick_msg_parse(packet, len, &msg_id, &js);
    if (st != CMD_SUCCESS)
    {
        ESP_LOGW(TAG, "%s: Joystick parse failed: %d", mod->name, st);
        return;
    }

    // 基于msg_id的简易丢包检测
    if (s_last_rx_msg_id != 0 && msg_id > s_last_rx_msg_id + 1)
    {
        ESP_LOGW(TAG, "%s: Packet loss: expect id %lu, got %lu (lost %lu)",
                 mod->name,
                 (unsigned long)(s_last_rx_msg_id + 1), (unsigned long)msg_id,
                 (unsigned long)(msg_id - s_last_rx_msg_id - 1));
    }
    s_last_rx_msg_id = msg_id;

    ESP_LOGI(TAG, "%s: >>> Joystick[%lu]: X=%-5d Y=%-5d BTN=0x%02X",
             mod->name, (unsigned long)msg_id, js.x, js.y, js.buttons);
    // TODO: 执行机构控制,如 motor_set_speed(js.x, js.y);

    // 链路监督:刷新时间戳;失联→恢复的状态迁移在此完成
    s_last_joystick_tick = xTaskGetTickCount();
    if (!s_link_up)
    {
        s_link_up = true;
        failsafe_release();
    }

#if ENABLE_LINK_ACK
    // 回ACK:此时发送方已发完(半双工无线电已空闲),直接回传即可。
    // 本函数运行在接收任务上下文中,与该模组的其他发送天然串行。
    send_link_ack(mod, msg_id);
#endif
}

/**
 * @brief 组帧层回调:每个CRC通过的完整cmd包到达这里,按pkt_type分发
 */
static void on_cmd_packet(const uint8_t *packet, uint16_t len, void *ctx)
{
    lora_module_t *mod = (lora_module_t *)ctx; // 由frame_parser_feed透传
    uint8_t pkt_type = unknown_pkt_type;
    if (cmd_extract_pkt_type(packet, len, &pkt_type) != CMD_SUCCESS)
    {
        return;
    }

    switch ((cmd_pkt_type_t)pkt_type)
    {
    case joystick_pkt_type:
        handle_joystick_pkt(mod, packet, len);
        break;

    case link_ack_pkt_type:
    {
        uint32_t acked_id = 0;
        cmd_extract_pkt_msg_id(packet, len, &acked_id);
        if (s_ack_miss_streak >= 5)
        {
            ESP_LOGW(TAG, "%s: === LINK RESTORED after %lu misses ===",
                     mod->name, (unsigned long)s_ack_miss_streak);
        }
        s_last_ack_id = acked_id;
        s_ack_miss_streak = 0;
        ESP_LOGI(TAG, "%s: <<< ACK[%lu] link OK", mod->name, (unsigned long)acked_id);
        break;
    }

    case heart_beat_pkt_type:
    {
        uint32_t msg_id = 0;
        cmd_extract_pkt_msg_id(packet, len, &msg_id);
        ESP_LOGI(TAG, "%s: >>> Heartbeat msg_id=%lu", mod->name, (unsigned long)msg_id);
        break;
    }

        // 新消息类型在此追加case,并在对应xxx_msg.c实现parse

    default:
        ESP_LOGW(TAG, "%s: Unknown pkt_type 0x%02X (len=%u)", mod->name, pkt_type, len);
        break;
    }
}

/* ===================== 通用接收任务(每模组一个实例) ===================== */

static void lora_rx_event_task(void *pvParameters)
{
    lora_rx_ctx_t *ctx = (lora_rx_ctx_t *)pvParameters;
    lora_module_t *mod = ctx->mod;
    uart_event_t event;
    uint8_t *rx_buf = (uint8_t *)malloc(LORA_UART_BUF_SIZE);
    if (rx_buf == NULL)
    {
        ESP_LOGE(TAG, "%s: Failed to allocate RX buffer", mod->name);
        vTaskDelete(NULL);
        return;
    }

    // 等待本机所有模组配置完成,防止AT应答被本任务抢走
    ESP_LOGI(TAG, "%s: RX Task waiting for config done...", mod->name);
    xEventGroupWaitBits(sys_evt_group, EVT_ALL_CONFIG_DONE, pdFALSE, pdTRUE, portMAX_DELAY);

    uart_flush_input(mod->uart_num);
    xQueueReset(*mod->evt_queue);
    frame_parser_init(&ctx->parser, on_cmd_packet);
    ESP_LOGI(TAG, "%s: RX Task Start", mod->name);

    for (;;)
    {
        if (xQueueReceive(*mod->evt_queue, &event, portMAX_DELAY))
        {
            switch (event.type)
            {
            case UART_DATA:
            {
                int rd = uart_read_bytes(mod->uart_num, rx_buf, event.size, portMAX_DELAY);
                if (rd <= 0)
                    break;

                // 剥离LoRa传输头(实测本批模组定点模式接收端输出保留
                // 地址2+信道1头);若固件输出纯载荷,整段进解析器,
                // SOF同步会自动对齐,前置杂散字节计入sync_drops
                const uint8_t *payload = rx_buf;
                size_t payload_len = (size_t)rd;
                if (mod->mode == 1 && rd >= 3 &&
                    rx_buf[0] == mod->addr_h && rx_buf[1] == mod->addr_l)
                {
                    payload = &rx_buf[3];
                    payload_len = (size_t)rd - 3;
                }

                ESP_LOGD(TAG, "%s: Recv %d bytes, feed %d to frame parser",
                         mod->name, rd, (int)payload_len);
                frame_parser_feed(&ctx->parser, payload, payload_len, mod);
                break;
            }
            case UART_FIFO_OVF:
            case UART_BUFFER_FULL:
                uart_flush_input(mod->uart_num);
                xQueueReset(*mod->evt_queue);
                frame_parser_init(&ctx->parser, NULL); // 传NULL保留原回调,仅复位状态
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

/* ===================== 接收侧:链路看门狗任务 ===================== */
#if HAS_RX_MODULE

/**
 * @brief 周期巡检"距上一包摇杆数据多久了",超时触发failsafe。
 * 独立小任务而非嵌在接收任务里,因为接收任务阻塞在队列上——
 * 恰恰是"什么都收不到"的时候它最不可能醒来做检查。
 */
static void link_watchdog_task(void *pvParameters)
{
    (void)pvParameters;
    xEventGroupWaitBits(sys_evt_group, EVT_ALL_CONFIG_DONE, pdFALSE, pdTRUE, portMAX_DELAY);
    ESP_LOGI(TAG, "Link watchdog start (timeout %dms)", LINK_TIMEOUT_MS);

    for (;;)
    {
        vTaskDelay(pdMS_TO_TICKS(LINK_WATCHDOG_CHECK_MS));
        if (s_link_up &&
            (xTaskGetTickCount() - s_last_joystick_tick) > pdMS_TO_TICKS(LINK_TIMEOUT_MS))
        {
            s_link_up = false;
            failsafe_activate();
            // 之后保持failsafe状态,直到handle_joystick_pkt收到新包才恢复
        }
    }
}
#endif // HAS_RX_MODULE

/* ===================== 发送侧:摇杆周期发送任务 ===================== */
#if HAS_TX_MODULE

#define ACK_MISS_LINK_DOWN 5 // 连续N包未ACK判定链路异常

static void joystick_tx_task(void *pvParameters)
{
    lora_module_t *mod = (lora_module_t *)pvParameters;
    joystick_state_t js_state;
    joystick_msg_t js_msg;
    uint8_t cmd_buf[CMD_MAX_PACKET_LEN];
    uint8_t frame_buf[FRAME_MAX_PAYLOAD + FRAME_OVERHEAD];
    uint32_t msg_id = 1; // 从1开始,0留给"未收到过"语义

    ESP_LOGI(TAG, "%s: Joystick TX Task waiting for config done...", mod->name);
    xEventGroupWaitBits(sys_evt_group, EVT_ALL_CONFIG_DONE, pdFALSE, pdTRUE, portMAX_DELAY);

    joystick_init();
    ESP_LOGI(TAG, "%s: Joystick TX Task Start, interval %dms, ACK %s",
             mod->name, JOYSTICK_SEND_INTERVAL_MS, ENABLE_LINK_ACK ? "ON" : "OFF");

    for (;;)
    {
        if (joystick_read(&js_state))
        {
#if ENABLE_LINK_ACK
            // 检查上一包是否已被ACK(留了整整一个发送周期,足够往返)
            if (msg_id > 1 && s_last_ack_id < msg_id - 1)
            {
                s_ack_miss_streak++;
                ESP_LOGW(TAG, "%s: pkt[%lu] NOT acked (miss streak %lu)",
                         mod->name, (unsigned long)(msg_id - 1),
                         (unsigned long)s_ack_miss_streak);
                if (s_ack_miss_streak >= ACK_MISS_LINK_DOWN)
                {
                    ESP_LOGE(TAG, "%s: LINK DOWN? %lu consecutive packets unacked",
                             mod->name, (unsigned long)s_ack_miss_streak);
                }
            }
#endif
            js_msg.x = js_state.x;
            js_msg.y = js_state.y;
            js_msg.buttons = js_state.buttons;

            uint16_t cmd_len = 0;
            cmd_status_t st = joystick_msg_write(cmd_buf, sizeof(cmd_buf), &cmd_len,
                                                 msg_id, &js_msg);
            if (st == CMD_SUCCESS)
            {
                int frame_len = frame_encode(cmd_buf, (uint8_t)cmd_len,
                                             frame_buf, sizeof(frame_buf));
                if (frame_len > 0)
                {
                    ESP_LOGI(TAG, "%s: TX Joystick[%lu] X=%-5d Y=%-5d BTN=0x%02X (cmd %uB frame %dB)",
                             mod->name, (unsigned long)msg_id, js_msg.x, js_msg.y,
                             js_msg.buttons, cmd_len, frame_len);
                    lora_send_bytes(mod, frame_buf, (size_t)frame_len);
                    msg_id++;
                }
            }
            else
            {
                ESP_LOGE(TAG, "%s: joystick_msg_write failed: %d", mod->name, st);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(JOYSTICK_SEND_INTERVAL_MS));
    }
}
#endif // HAS_TX_MODULE

/* ===================== 上电AT配置任务(每模组一个) ===================== */

static void lora_module_at_config_task(void *pvParameters)
{
    lora_module_t *mod = (lora_module_t *)pvParameters;

    ESP_LOGI(TAG, "Waiting for %s to power on...", mod->name);
    vTaskDelay(pdMS_TO_TICKS(3000));
    clear_uart_buffer(mod->uart_num);

    bool ok = configure_and_verify(mod);
    ESP_LOGI(TAG, "==== Config Result: %s=%s ====", mod->name, ok ? "PASS" : "FAIL");

#if HAS_RX_MODULE
    if (mod == &g_rx_module)
    {
        xEventGroupSetBits(sys_evt_group, EVT_RX_CONFIG_DONE);
        vTaskDelete(NULL);
        return;
    }
#endif
#if HAS_TX_MODULE
    if (mod == &g_tx_module)
    {
        xEventGroupSetBits(sys_evt_group, EVT_TX_CONFIG_DONE);
    }
#endif
    vTaskDelete(NULL);
}

/* ===================== 主入口 ===================== */

void app_main(void)
{
    esp_log_level_set(TAG, ESP_LOG_INFO);

    ESP_LOGI(TAG, "==== DEVICE_ROLE=%d (TX_MOD=%d RX_MOD=%d) ACK=%d ====",
             DEVICE_ROLE, HAS_TX_MODULE, HAS_RX_MODULE, ENABLE_LINK_ACK);

    sys_evt_group = xEventGroupCreate();
    if (sys_evt_group == NULL)
    {
        ESP_LOGE(TAG, "Failed to create event group");
        return;
    }

    /* 硬件初始化 + 配置任务(独立UART并行配置)
     * 每个模组都创建接收任务:TX模组也要收(ACK回传);
     * 各业务任务通过事件组等待 EVT_ALL_CONFIG_DONE 后才启动。
     */
#if HAS_TX_MODULE
#if AUX_CONNECTED
    aux_gpio_init(g_tx_module.aux_pin);
#endif
    lora_uart_init(&g_tx_module, &lora_tx_queue);
    xTaskCreate(lora_module_at_config_task, "lora_tx_cfg", 4096, &g_tx_module, 12, NULL);
    xTaskCreate(lora_rx_event_task, "rx_txmod", 4096, &s_tx_mod_ctx, 11, NULL);
#endif

#if HAS_RX_MODULE
#if AUX_CONNECTED
    aux_gpio_init(g_rx_module.aux_pin);
#endif
    lora_uart_init(&g_rx_module, &lora_rx_queue);
    xTaskCreate(lora_module_at_config_task, "lora_rx_cfg", 4096, &g_rx_module, 12, NULL);
    xTaskCreate(lora_rx_event_task, "rx_rxmod", 4096, &s_rx_mod_ctx, 11, NULL);
    xTaskCreate(link_watchdog_task, "link_wdt", 2048, NULL, 9, NULL);
#endif

#if HAS_TX_MODULE
    xTaskCreate(joystick_tx_task, "joystick_tx", 4096, &g_tx_module, 10, NULL);
#endif
}