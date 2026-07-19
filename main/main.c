#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "board_config.h"
#include "nrf24l01.h"
#include "comm_frame.h"
#include "cmds.h"
#include "joystick.h"

static const char *TAG = "NRF_MAIN";

/*
 * ============================================================
 *  组装层:按 board_config.h 中的 DEVICE_ROLE 拼装任务。
 *
 *  协议栈(自上而下):
 *    cmds       应用协议(type+msg_id+TLV字段)
 *    comm_frame 链路组帧(SOF+LEN+CRC8)
 *    nrf24l01   2.4G传输(SPI接口,nRF24L01+PA+LNA)
 *
 *  通信策略:
 *    - 摇杆数据:无ACK高频发送,高实时性
 *    - 心跳包:接收端回ACK,检测链路状态
 *
 *  【本版改动】
 *  1. NRF_DIAG_ENABLE=1 时启动诊断任务:寄存器dump + 载波测试
 *  2. poll任务超时分支补捞RX FIFO:IRQ线未接/丢边沿时仍能收包
 *  3. 超时计数移入 radio_rx_ctx_t(原为函数级static,两任务共享)
 * ============================================================
 */

/* 【诊断开关】1=启动后自动跑一轮 寄存器dump+载波测试,定位后改回0 */
#define NRF_DIAG_ENABLE 1

/* 诊断期间暂停发送任务(载波测试时发包会掐断载波,导致CD误判) */
static volatile bool s_diag_pause = false;

/* ---------- 同步对象 ---------- */
static EventGroupHandle_t sys_evt_group = NULL;
#define EVT_TX_INIT_DONE BIT0
#define EVT_RX_INIT_DONE BIT1

#define EVT_ALL_INIT_DONE \
    ((HAS_TX_MODULE ? EVT_TX_INIT_DONE : 0) | (HAS_RX_MODULE ? EVT_RX_INIT_DONE : 0))

/* ---------- 每模组接收上下文 ---------- */
typedef struct
{
    nrf24_module_t *mod;
    frame_parser_t parser;
    uint32_t irq_timeout_count; // 【改】每模组独立计数,不再共享static
} radio_rx_ctx_t;

/* ===================== 模块描述符 ===================== */

#if HAS_RX_MODULE
static nrf24_module_t g_rx_module = {
    .name = "RX-Mod",
    .spi_host = NRF24_RX_SPI_HOST,
    .ce_pin = NRF24_RX_CE_PIN,
    .csn_pin = NRF24_RX_CSN_PIN,
    .sck_pin = NRF24_RX_SCK_PIN,
    .mosi_pin = NRF24_RX_MOSI_PIN,
    .miso_pin = NRF24_RX_MISO_PIN,
    .irq_pin = NRF24_RX_IRQ_PIN,
    .channel = NRF24_CHANNEL,
    .speed = NRF24_SPEED,
    .power = NRF24_POWER,
    .rx_addr = NRF24_RECEIVER_ADDR,
    .tx_addr = NRF24_CONTROLLER_ADDR,
    .payload_len = NRF24_PAYLOAD_LEN,
    .rx_callback = NULL,
};
static radio_rx_ctx_t s_rx_mod_ctx = {.mod = &g_rx_module};
#endif

#if HAS_TX_MODULE
static nrf24_module_t g_tx_module = {
    .name = "TX-Mod",
    .spi_host = NRF24_TX_SPI_HOST,
    .ce_pin = NRF24_TX_CE_PIN,
    .csn_pin = NRF24_TX_CSN_PIN,
    .sck_pin = NRF24_TX_SCK_PIN,
    .mosi_pin = NRF24_TX_MOSI_PIN,
    .miso_pin = NRF24_TX_MISO_PIN,
    .irq_pin = NRF24_TX_IRQ_PIN,
    .channel = NRF24_CHANNEL,
    .speed = NRF24_SPEED,
    .power = NRF24_POWER,
    .rx_addr = NRF24_CONTROLLER_ADDR,
    .tx_addr = NRF24_RECEIVER_ADDR,
    .payload_len = NRF24_PAYLOAD_LEN,
    .rx_callback = NULL,
};
static radio_rx_ctx_t s_tx_mod_ctx = {.mod = &g_tx_module};
#endif

/* ===================== 消息处理 ===================== */

static uint32_t s_last_rx_msg_id = 0;

/* ---------- 链路状态(接收侧:基于摇杆包超时) ---------- */
static volatile bool s_link_up = false;
static volatile TickType_t s_last_joystick_tick = 0;

#if HAS_TX_MODULE
/* ---------- 链路状态(发送侧:基于心跳ACK超时) ----------
 * heartbeat_tx_task 每拍发送前检查上一拍是否已被ACK;
 * 连续 HEARTBEAT_MISS_LINK_DOWN 拍无ACK → 判定TX侧断链。
 * ACK到达(on_cmd_packet)立即清零miss计数并恢复链路状态。
 */
static volatile uint32_t s_hb_last_acked_id = 0; // 已被ACK的最大心跳msg_id
static volatile uint32_t s_hb_miss_count = 0;    // 连续无ACK计数
static volatile bool s_tx_link_up = false;       // 发送侧链路状态

static void tx_link_down(void)
{
    ESP_LOGE(TAG, "!!! TX LINK DOWN: %d heartbeats unacked, receiver lost !!!",
             HEARTBEAT_MISS_LINK_DOWN);
    // 在此挂接发送侧断链动作:如状态LED、震动提示、停止高频发送等
}

static void tx_link_up(void)
{
    ESP_LOGW(TAG, "=== TX LINK UP: heartbeat ACK received ===");
}
#endif

static void failsafe_activate(void)
{
    ESP_LOGE(TAG, "!!! FAILSAFE: link lost, outputs neutralized !!!");
}

static void failsafe_release(void)
{
    ESP_LOGW(TAG, "=== LINK RESTORED, resume control ===");
}

static void handle_joystick_pkt(nrf24_module_t *mod, const uint8_t *packet, uint16_t len)
{
    joystick_msg_t js = {0};
    uint32_t msg_id = 0;

    cmd_status_t st = joystick_msg_parse(packet, len, &msg_id, &js);
    if (st != CMD_SUCCESS)
    {
        ESP_LOGW(TAG, "%s: Joystick parse failed: %d", mod->name, st);
        return;
    }

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

    s_last_joystick_tick = xTaskGetTickCount();
    if (!s_link_up)
    {
        s_link_up = true;
        failsafe_release();
    }
}

static void on_cmd_packet(const uint8_t *packet, uint16_t len, void *ctx)
{
    nrf24_module_t *mod = (nrf24_module_t *)ctx;
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

    case heart_beat_pkt_type:
    {
        uint32_t msg_id = 0;
        cmd_extract_pkt_msg_id(packet, len, &msg_id);
        ESP_LOGI(TAG, "%s: >>> Heartbeat msg_id=%lu", mod->name, (unsigned long)msg_id);

        uint8_t ack_buf[16];
        uint8_t ack_frame_buf[16 + FRAME_OVERHEAD];
        uint16_t ack_len = 0;
        if (heartbeat_ack_write(ack_buf, sizeof(ack_buf), &ack_len, msg_id) == CMD_SUCCESS)
        {
            int frame_len = frame_encode(ack_buf, ack_len, ack_frame_buf, sizeof(ack_frame_buf));
            if (frame_len > 0)
            {
                ESP_LOGI(TAG, "%s: <<< Heartbeat ACK msg_id=%lu", mod->name, (unsigned long)msg_id);
                nrf24_send_packet(mod, ack_frame_buf, (uint8_t)frame_len);
            }
        }
        break;
    }

    case heart_beat_ack_pkt_type:
    {
        uint32_t acked_msg_id = 0;
        cmd_extract_pkt_msg_id(packet, len, &acked_msg_id);
        ESP_LOGI(TAG, "%s: <<< Heartbeat ACK received for msg_id=%lu", mod->name, (unsigned long)acked_msg_id);
#if HAS_TX_MODULE
        if (acked_msg_id > s_hb_last_acked_id)
        {
            s_hb_last_acked_id = acked_msg_id;
        }
        s_hb_miss_count = 0;
        if (!s_tx_link_up)
        {
            s_tx_link_up = true;
            tx_link_up();
        }
#endif
        break;
    }

    default:
        ESP_LOGW(TAG, "%s: Unknown pkt_type 0x%02X (len=%u)", mod->name, pkt_type, len);
        break;
    }
}

/* ===================== 通用接收任务(每模组一个) ===================== */

/** @brief 捞空RX FIFO,返回是否捞到过包 */
static bool drain_rx_fifo(radio_rx_ctx_t *ctx, uint8_t *rx_buf)
{
    nrf24_module_t *mod = ctx->mod;
    uint8_t rx_len;
    bool got = false;

    while (!nrf24_rx_fifo_empty(mod))
    {
        if (nrf24_read_packet(mod, rx_buf, &rx_len) != ESP_OK)
        {
            break;
        }
        ESP_LOGI(TAG, "%s: Recv %u bytes", mod->name, rx_len);
        frame_parser_feed(&ctx->parser, rx_buf, rx_len, mod);
        got = true;
    }
    return got;
}

static void nrf_rx_poll_task(void *pvParameters)
{
    radio_rx_ctx_t *ctx = (radio_rx_ctx_t *)pvParameters;
    nrf24_module_t *mod = ctx->mod;
    uint8_t rx_buf[NRF_MAX_PAYLOAD];

    ESP_LOGI(TAG, "%s: RX Task waiting for init done...", mod->name);
    xEventGroupWaitBits(sys_evt_group, EVT_ALL_INIT_DONE, pdFALSE, pdTRUE, portMAX_DELAY);

    frame_parser_init(&ctx->parser, on_cmd_packet);
    nrf24_set_rx_mode(mod);
    ESP_LOGI(TAG, "%s: RX Task Start (IRQ-driven, timeout-drain fallback)", mod->name);

    for (;;)
    {
        BaseType_t irq_result = nrf24_wait_irq(mod, pdMS_TO_TICKS(1000));
        if (irq_result != pdTRUE)
        {
            /* 【改】超时也补捞一次FIFO:IRQ线未接/边沿丢失时仍能收包。
             * 若在此路径收到包而IRQ从不触发,说明IRQ接线有问题。 */
            if (drain_rx_fifo(ctx, rx_buf))
            {
                ESP_LOGW(TAG, "%s: Got packet WITHOUT IRQ! Check IRQ wiring (GPIO%d)",
                         mod->name, mod->irq_pin);
                continue;
            }
            if (++ctx->irq_timeout_count % 5 == 0) // 每5秒打印一次
            {
                uint8_t status = nrf24_read_status(mod);
                uint8_t fifo_status = nrf24_read_reg(mod, NRF_REG_FIFO_STATUS);
                ESP_LOGW(TAG, "%s: IRQ timeout (count=%lu), STATUS=0x%02X, FIFO=0x%02X",
                         mod->name, (unsigned long)ctx->irq_timeout_count, status, fifo_status);
            }
            continue;
        }

        uint8_t status = nrf24_read_status(mod);
        ESP_LOGD(TAG, "%s: IRQ triggered, STATUS=0x%02X", mod->name, status);

        if (status & NRF_STATUS_RX_DR)
        {
            drain_rx_fifo(ctx, rx_buf);
            nrf24_clear_irq(mod, NRF_STATUS_RX_DR);
        }
        if (status & (NRF_STATUS_TX_DS | NRF_STATUS_MAX_RT))
        {
            nrf24_clear_irq(mod, NRF_STATUS_TX_DS | NRF_STATUS_MAX_RT);
        }
    }
}

/* ===================== 接收侧:链路看门狗 ===================== */
#if HAS_RX_MODULE

static void link_watchdog_task(void *pvParameters)
{
    (void)pvParameters;
    xEventGroupWaitBits(sys_evt_group, EVT_ALL_INIT_DONE, pdFALSE, pdTRUE, portMAX_DELAY);
    ESP_LOGI(TAG, "Link watchdog start (timeout %dms)", LINK_TIMEOUT_MS);

    for (;;)
    {
        vTaskDelay(pdMS_TO_TICKS(LINK_WATCHDOG_CHECK_MS));
        if (s_link_up &&
            (xTaskGetTickCount() - s_last_joystick_tick) > pdMS_TO_TICKS(LINK_TIMEOUT_MS))
        {
            s_link_up = false;
            failsafe_activate();
        }
    }
}
#endif

/* ===================== 发送侧:摇杆高频发送 ===================== */
#if HAS_TX_MODULE

static void joystick_tx_task(void *pvParameters)
{
    nrf24_module_t *mod = (nrf24_module_t *)pvParameters;
    joystick_state_t js_state;
    joystick_msg_t js_msg;
    uint8_t cmd_buf[CMD_MAX_PACKET_LEN];
    uint8_t frame_buf[FRAME_MAX_PAYLOAD + FRAME_OVERHEAD];
    uint32_t msg_id = 1;

    ESP_LOGI(TAG, "%s: Joystick TX Task waiting for init done...", mod->name);
    xEventGroupWaitBits(sys_evt_group, EVT_ALL_INIT_DONE, pdFALSE, pdTRUE, portMAX_DELAY);

    joystick_init();
    ESP_LOGI(TAG, "%s: Joystick TX Task Start, interval %dms",
             mod->name, JOYSTICK_SEND_INTERVAL_MS);

    for (;;)
    {
        if (s_diag_pause)
        {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }
        if (joystick_read(&js_state))
        {
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
                    ESP_LOGI(TAG, "%s: TX Joystick[%lu] X=%-5d Y=%-5d BTN=0x%02X",
                             mod->name, (unsigned long)msg_id, js_msg.x, js_msg.y, js_msg.buttons);
                    if (nrf24_send_packet(mod, frame_buf, (uint8_t)frame_len) != ESP_OK)
                    {
                        ESP_LOGW(TAG, "%s: Joystick send failed", mod->name);
                    }
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
#endif

/* ===================== 发送侧:心跳包 ===================== */
#if HAS_TX_MODULE

static void heartbeat_tx_task(void *pvParameters)
{
    nrf24_module_t *mod = (nrf24_module_t *)pvParameters;
    uint8_t cmd_buf[16];
    uint8_t frame_buf[16 + FRAME_OVERHEAD];
    uint32_t msg_id = 1;

    ESP_LOGI(TAG, "%s: Heartbeat TX Task waiting for init done...", mod->name);
    xEventGroupWaitBits(sys_evt_group, EVT_ALL_INIT_DONE, pdFALSE, pdTRUE, portMAX_DELAY);

    ESP_LOGI(TAG, "%s: Heartbeat TX Task Start, interval %dms",
             mod->name, HEARTBEAT_INTERVAL_MS);

    for (;;)
    {
        if (s_diag_pause)
        {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        /* ---- ACK超时检测:发送第N拍前,检查第N-1拍是否已被ACK ----
         * (上一拍发出后已过整个 HEARTBEAT_INTERVAL_MS,ACK时间充裕) */
        if (msg_id > 1 && s_hb_last_acked_id < msg_id - 1)
        {
            s_hb_miss_count++;
            ESP_LOGW(TAG, "%s: Heartbeat[%lu] no ACK (miss %lu/%d)",
                     mod->name, (unsigned long)(msg_id - 1),
                     (unsigned long)s_hb_miss_count, HEARTBEAT_MISS_LINK_DOWN);
            if (s_tx_link_up && s_hb_miss_count >= HEARTBEAT_MISS_LINK_DOWN)
            {
                s_tx_link_up = false;
                tx_link_down();
            }
        }

        uint16_t cmd_len = 0;
        cmd_status_t st = heartbeat_write(cmd_buf, sizeof(cmd_buf), &cmd_len, msg_id);
        if (st == CMD_SUCCESS)
        {
            int frame_len = frame_encode(cmd_buf, (uint8_t)cmd_len,
                                         frame_buf, sizeof(frame_buf));
            if (frame_len > 0)
            {
                ESP_LOGI(TAG, "%s: TX Heartbeat[%lu]",
                         mod->name, (unsigned long)msg_id);
                nrf24_send_packet(mod, frame_buf, (uint8_t)frame_len);
                msg_id++;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(HEARTBEAT_INTERVAL_MS));
    }
}
#endif

/* ===================== 诊断任务(NRF_DIAG_ENABLE=1 时运行) ===================== */
#if NRF_DIAG_ENABLE && HAS_TX_MODULE && HAS_RX_MODULE

static void diag_task(void *pvParameters)
{
    (void)pvParameters;
    xEventGroupWaitBits(sys_evt_group, EVT_ALL_INIT_DONE, pdFALSE, pdTRUE, portMAX_DELAY);
    vTaskDelay(pdMS_TO_TICKS(3000)); // 等poll任务进入监听态

    ESP_LOGW(TAG, "######## DIAG START (TX tasks paused) ########");

    /* 【v2】暂停发送任务:上一版载波开启20ms就被摇杆发包掐断,
     * CD全程测的是"无载波",结论无效 */
    s_diag_pause = true;
    vTaskDelay(pdMS_TO_TICKS(500)); // 等在途发送结束

    /* 第1步:双方寄存器dump(与期望值逐项比对) */
    nrf24_dump_regs(&g_tx_module);
    nrf24_dump_regs(&g_rx_module);

    /* 第2步:TX恒定载波,RX测CD → 验证RF物理通路
     * CD=1: 电波到达RX芯片,RF通路OK,问题在灵敏度/饱和/配置细节
     * CD=0: 电波没到,查供电/天线/CE线/模组本身 */
    nrf24_carrier_test_start(&g_tx_module);
    vTaskDelay(pdMS_TO_TICKS(100));

    /* 中途核验载波确实还开着:RF_SETUP 应为 0x9E (CONT_WAVE|PLL_LOCK|2M|0dBm) */
    ESP_LOGI(TAG, "DIAG: TX-Mod RF_SETUP during carrier = 0x%02X (expect 0x9E)",
             nrf24_read_reg(&g_tx_module, NRF_REG_RF_SETUP));

    int cd_hits = 0;
    for (int i = 0; i < 20; i++)
    {
        vTaskDelay(pdMS_TO_TICKS(200));
        uint8_t cd = nrf24_carrier_detect(&g_rx_module);
        cd_hits += cd;
        ESP_LOGI(TAG, "DIAG: RX-Mod CD=%d (%d/20)", cd, i + 1);
    }

    /* 再核验一次:测试全程载波未被打断 */
    ESP_LOGI(TAG, "DIAG: TX-Mod RF_SETUP after test = 0x%02X (expect 0x9E)",
             nrf24_read_reg(&g_tx_module, NRF_REG_RF_SETUP));

    nrf24_carrier_test_stop(&g_tx_module);
    nrf24_set_rx_mode(&g_tx_module); // 恢复TX-Mod监听
    s_diag_pause = false;            // 恢复发送任务

    ESP_LOGW(TAG, "######## DIAG RESULT: CD hits %d/20 → %s ########",
             cd_hits,
             cd_hits > 15  ? "RF PATH OK (查灵敏度/饱和)"
             : cd_hits > 0 ? "RF UNSTABLE (查供电去耦/天线)"
                           : "NO RF (查供电/天线/CE线/模组)");

    vTaskDelete(NULL);
}
#endif

/* ===================== 初始化任务(每模组一个) ===================== */

static void nrf_init_task(void *pvParameters)
{
    nrf24_module_t *mod = (nrf24_module_t *)pvParameters;

    ESP_LOGI(TAG, "Init %s...", mod->name);
    vTaskDelay(pdMS_TO_TICKS(500));

    esp_err_t err = nrf24_init(mod);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "==== Init Result: %s=FAIL ====", mod->name);
    }
    else
    {
        uint8_t status = nrf24_read_status(mod);
        ESP_LOGI(TAG, "==== Init Result: %s=OK (STATUS=0x%02X) ====", mod->name, status);
    }

#if HAS_RX_MODULE
    if (mod == &g_rx_module)
    {
        xEventGroupSetBits(sys_evt_group, EVT_RX_INIT_DONE);
        vTaskDelete(NULL);
        return;
    }
#endif
#if HAS_TX_MODULE
    if (mod == &g_tx_module)
    {
        xEventGroupSetBits(sys_evt_group, EVT_TX_INIT_DONE);
    }
#endif
    vTaskDelete(NULL);
}

/* ===================== 主入口 ===================== */

void app_main(void)
{
    esp_log_level_set(TAG, ESP_LOG_INFO);

    ESP_LOGI(TAG, "==== DEVICE_ROLE=%d (TX=%d RX=%d) RADIO=nRF24 ====",
             DEVICE_ROLE, HAS_TX_MODULE, HAS_RX_MODULE);

    sys_evt_group = xEventGroupCreate();
    if (sys_evt_group == NULL)
    {
        ESP_LOGE(TAG, "Failed to create event group");
        return;
    }

#if HAS_TX_MODULE
    xTaskCreate(nrf_init_task, "nrf_tx_init", 4096, &g_tx_module, 12, NULL);
    xTaskCreate(nrf_rx_poll_task, "rx_txmod", 4096, &s_tx_mod_ctx, 11, NULL);
#endif

#if HAS_RX_MODULE
    xTaskCreate(nrf_init_task, "nrf_rx_init", 4096, &g_rx_module, 12, NULL);
    xTaskCreate(nrf_rx_poll_task, "rx_rxmod", 4096, &s_rx_mod_ctx, 11, NULL);
    xTaskCreate(link_watchdog_task, "link_wdt", 2048, NULL, 9, NULL);
#endif

#if HAS_TX_MODULE
    xTaskCreate(joystick_tx_task, "joystick_tx", 4096, &g_tx_module, 10, NULL);
    xTaskCreate(heartbeat_tx_task, "heartbeat_tx", 4096, &g_tx_module, 10, NULL);
#endif

#if NRF_DIAG_ENABLE && HAS_TX_MODULE && HAS_RX_MODULE
    xTaskCreate(diag_task, "nrf_diag", 4096, NULL, 8, NULL);
#endif
}