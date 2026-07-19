#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "nrf24l01.h"

static const char *TAG = "NRF24";

#define CE_LOW(mod) gpio_set_level((mod)->ce_pin, 0)
#define CE_HIGH(mod) gpio_set_level((mod)->ce_pin, 1)

/* SPI 总线初始化状态:同一 host 只能初始化一次,多模块共享总线时由第一个模块完成 */
static bool s_spi_bus_inited[SOC_SPI_PERIPH_NUM] = {false};

/*
 * ============================================================
 *  【修复说明】
 *  1. 发送流程:原实现在CE脉冲后立即清TX_DS、FLUSH_TX并切回RX。
 *     CE脉冲后芯片还需 130µs TX settling + ~160µs 空中时间(2Mbps)
 *     才真正发完,原实现在 ~20µs 内就冲掉FIFO并改写CONFIG,
 *     每一包都在发射途中被中止 → 接收端永远收不到。
 *     现在:CE脉冲后轮询STATUS等待TX_DS(超时5ms),
 *     成功路径不再FLUSH_TX(发完后载荷自动出FIFO),仅失败时冲。
 *  2. 并发保护:同一模组被 joystick_tx / heartbeat_tx / rx_poll
 *     多个任务并发访问,SPI事务与模式切换必须串行化。
 *     所有公共接口内部加模组级互斥锁;内部 *_unlocked 原语不加锁。
 *  3. PWR_UP时序:掉电→Standby-I 需 Tpd2stby(≤1.5ms),
 *     首次上电时等待2ms再拉CE。
 * ============================================================
 */

/* ===================== IRQ 中断服务 ===================== */

static void IRAM_ATTR nrf_irq_isr(void *arg)
{
    nrf24_module_t *mod = (nrf24_module_t *)arg;
    BaseType_t hpw = pdFALSE;
    /* ISR 里只给信号量,不做 SPI 操作(共享总线不能在中断里访问) */
    xSemaphoreGiveFromISR(mod->irq_sem, &hpw);
    if (hpw)
        portYIELD_FROM_ISR();
}

BaseType_t nrf24_wait_irq(nrf24_module_t *mod, TickType_t timeout)
{
    return xSemaphoreTake(mod->irq_sem, timeout);
}

/* ===================== SPI 底层(不加锁,仅内部调用) ===================== */

static esp_err_t spi_xfer(const nrf24_module_t *mod, const uint8_t *tx_data,
                          uint8_t *rx_data, size_t len)
{
    spi_transaction_t t = {
        .length = len * 8,
        .tx_buffer = tx_data,
        .rx_buffer = rx_data,
    };
    return spi_device_polling_transmit(mod->spi_handle, &t);
}

static uint8_t nrf_read_reg(const nrf24_module_t *mod, uint8_t reg)
{
    uint8_t tx[2] = {(uint8_t)(NRF_CMD_R_REGISTER | reg), 0xFF};
    uint8_t rx[2];
    spi_xfer(mod, tx, rx, 2);
    return rx[1];
}

static void nrf_write_reg(const nrf24_module_t *mod, uint8_t reg, uint8_t val)
{
    uint8_t tx[2] = {(uint8_t)(NRF_CMD_W_REGISTER | reg), val};
    uint8_t rx[2];
    spi_xfer(mod, tx, rx, 2);
}

static void nrf_write_buf(const nrf24_module_t *mod, uint8_t reg,
                          const uint8_t *buf, uint8_t len)
{
    uint8_t tx[33];
    tx[0] = NRF_CMD_W_REGISTER | reg;
    memcpy(&tx[1], buf, len);
    uint8_t rx[33];
    spi_xfer(mod, tx, rx, len + 1);
}

static void nrf_read_buf(const nrf24_module_t *mod, uint8_t reg,
                         uint8_t *buf, uint8_t len)
{
    uint8_t tx[33];
    tx[0] = NRF_CMD_R_REGISTER | reg;
    memset(&tx[1], 0xFF, len);
    uint8_t rx[33];
    spi_xfer(mod, tx, rx, len + 1);
    memcpy(buf, &rx[1], len);
}

static uint8_t nrf_send_cmd(const nrf24_module_t *mod, uint8_t cmd)
{
    uint8_t tx = cmd;
    uint8_t rx;
    spi_xfer(mod, &tx, &rx, 1);
    return rx;
}

/* ===================== 内部原语(不加锁) ===================== */

/** @brief 首次 PWR_UP 后等待 Tpd2stby(掉电→Standby-I,≤1.5ms) */
static void power_up_wait_unlocked(nrf24_module_t *mod)
{
    if (!mod->powered_up)
    {
        vTaskDelay(pdMS_TO_TICKS(2));
        mod->powered_up = true;
    }
}

/**
 * @brief 进入接收模式(内部版,调用前必须已持锁)
 * @param flush_rx true=清空RX FIFO并清RX_DR(仅初次进入时用);
 *                 false=发送后返回RX,保留FIFO中未读包与RX_DR标志
 */
static void enter_rx_unlocked(nrf24_module_t *mod, bool flush_rx)
{
    CE_LOW(mod);

    /* PWR_UP=1, PRIM_RX=1 */
    uint8_t cfg = NRF_CONFIG_EN_CRC | NRF_CONFIG_CRCO |
                  NRF_CONFIG_PWR_UP | NRF_CONFIG_PRIM_RX;
    nrf_write_reg(mod, NRF_REG_CONFIG, cfg);
    power_up_wait_unlocked(mod);

    if (flush_rx)
    {
        nrf_send_cmd(mod, NRF_CMD_FLUSH_RX);
        nrf_write_reg(mod, NRF_REG_STATUS,
                      NRF_STATUS_RX_DR | NRF_STATUS_TX_DS | NRF_STATUS_MAX_RT);
    }
    else
    {
        /* 只清发送侧标志,不动RX_DR/RX FIFO,避免丢已到达的包 */
        nrf_write_reg(mod, NRF_REG_STATUS,
                      NRF_STATUS_TX_DS | NRF_STATUS_MAX_RT);
    }

    /* CE=1: 开启 LNA,进入接收监听 */
    CE_HIGH(mod);
    mod->is_rx_mode = true;
}

/** @brief 进入发送待命模式(内部版,调用前必须已持锁) */
static void enter_tx_unlocked(nrf24_module_t *mod)
{
    CE_LOW(mod);

    /* PWR_UP=1, PRIM_RX=0 */
    uint8_t cfg = NRF_CONFIG_EN_CRC | NRF_CONFIG_CRCO | NRF_CONFIG_PWR_UP;
    nrf_write_reg(mod, NRF_REG_CONFIG, cfg);
    power_up_wait_unlocked(mod);

    nrf_write_reg(mod, NRF_REG_STATUS, NRF_STATUS_TX_DS | NRF_STATUS_MAX_RT);
    mod->is_rx_mode = false;
}

static bool rx_fifo_empty_unlocked(const nrf24_module_t *mod)
{
    return (nrf_read_reg(mod, NRF_REG_FIFO_STATUS) & NRF_FIFO_RX_EMPTY) != 0;
}

/* ===================== 公共接口(加锁) ===================== */

uint8_t nrf24_read_status(nrf24_module_t *mod)
{
    xSemaphoreTake(mod->lock, portMAX_DELAY);
    uint8_t st = nrf_send_cmd(mod, NRF_CMD_NOP);
    xSemaphoreGive(mod->lock);
    return st;
}

uint8_t nrf24_read_reg(nrf24_module_t *mod, uint8_t reg)
{
    xSemaphoreTake(mod->lock, portMAX_DELAY);
    uint8_t val = nrf_read_reg(mod, reg);
    xSemaphoreGive(mod->lock);
    return val;
}

bool nrf24_tx_fifo_empty(nrf24_module_t *mod)
{
    xSemaphoreTake(mod->lock, portMAX_DELAY);
    uint8_t fifo = nrf_read_reg(mod, NRF_REG_FIFO_STATUS);
    xSemaphoreGive(mod->lock);
    return (fifo & NRF_FIFO_TX_EMPTY) != 0;
}

bool nrf24_rx_fifo_empty(nrf24_module_t *mod)
{
    xSemaphoreTake(mod->lock, portMAX_DELAY);
    bool empty = rx_fifo_empty_unlocked(mod);
    xSemaphoreGive(mod->lock);
    return empty;
}

static void nrf24_config_rf(nrf24_module_t *mod)
{
    uint8_t rf_setup = 0;

    switch (mod->speed)
    {
    case NRF24_SPEED_250K:
        rf_setup |= NRF_RF_SETUP_RF_DR_LOW;
        break;
    case NRF24_SPEED_1M:
        break;
    case NRF24_SPEED_2M:
        rf_setup |= NRF_RF_SETUP_RF_DR_HIGH;
        break;
    }

    switch (mod->power)
    {
    case NRF24_PWR_MIN18DBM:
        rf_setup |= NRF_RF_PWR_MIN18DBM;
        break;
    case NRF24_PWR_MIN12DBM:
        rf_setup |= NRF_RF_PWR_MIN12DBM;
        break;
    case NRF24_PWR_MIN6DBM:
        rf_setup |= NRF_RF_PWR_MIN6DBM;
        break;
    case NRF24_PWR_0DBM:
        rf_setup |= NRF_RF_PWR_0DBM;
        break;
    }

    nrf_write_reg(mod, NRF_REG_RF_SETUP, rf_setup);
    nrf_write_reg(mod, NRF_REG_RF_CH, mod->channel & 0x7F);
}

esp_err_t nrf24_init(nrf24_module_t *mod)
{
    esp_err_t ret;
    if (mod == NULL)
        return ESP_ERR_INVALID_ARG;

    ESP_LOGI(TAG, "%s: Init (ch=%d, speed=%d, power=%d)", mod->name,
             mod->channel, mod->speed, mod->power);

    /* 创建 IRQ 信号量 + 模组互斥锁 */
    mod->irq_sem = xSemaphoreCreateBinary();
    mod->lock = xSemaphoreCreateMutex();
    if (mod->irq_sem == NULL || mod->lock == NULL)
    {
        ESP_LOGE(TAG, "%s: Failed to create semaphore/mutex", mod->name);
        return ESP_ERR_NO_MEM;
    }
    mod->powered_up = false;
    mod->irq_wire_chk = 0;

    /* CE/CSN 引脚配置 */
    gpio_config_t io_out_cfg = {
        .pin_bit_mask = (1ULL << mod->ce_pin) | (1ULL << mod->csn_pin),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_out_cfg);
    CE_LOW(mod);
    gpio_set_level(mod->csn_pin, 1);

    /* IRQ 引脚:输入+上拉,下降沿触发(低电平有效) */
    gpio_config_t io_irq_cfg = {
        .pin_bit_mask = (1ULL << mod->irq_pin),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE,
    };
    gpio_config(&io_irq_cfg);

    /* 安装 GPIO 中断服务(重复调用安全) */
    esp_err_t isr_ret = gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
    if (isr_ret != ESP_OK && isr_ret != ESP_ERR_INVALID_STATE)
    {
        ESP_LOGE(TAG, "%s: ISR service install failed: %d", mod->name, isr_ret);
    }

    gpio_isr_handler_add(mod->irq_pin, nrf_irq_isr, mod);

    /* SPI 总线初始化(共享总线只初始化一次) */
    if (!s_spi_bus_inited[mod->spi_host])
    {
        spi_bus_config_t bus_cfg = {
            .mosi_io_num = mod->mosi_pin,
            .miso_io_num = mod->miso_pin,
            .sclk_io_num = mod->sck_pin,
            .quadwp_io_num = -1,
            .quadhd_io_num = -1,
            .max_transfer_sz = 64,
        };
        ret = spi_bus_initialize(mod->spi_host, &bus_cfg, SPI_DMA_CH_AUTO);
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "%s: SPI bus init failed: %d", mod->name, ret);
            return ret;
        }
        s_spi_bus_inited[mod->spi_host] = true;
        ESP_LOGI(TAG, "%s: SPI bus (host=%d) initialized", mod->name, mod->spi_host);
    }

    /* SPI 设备 - nRF24L01 用 SPI 模式 0 (CPOL=0, CPHA=0) */
    spi_device_interface_config_t dev_cfg = {
        .clock_speed_hz = 8 * 1000 * 1000,
        .mode = 0,
        .spics_io_num = mod->csn_pin,
        .queue_size = 8,
    };
    ret = spi_bus_add_device(mod->spi_host, &dev_cfg, &mod->spi_handle);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "%s: SPI add device failed: %d", mod->name, ret);
        return ret;
    }

    vTaskDelay(pdMS_TO_TICKS(100));

    /* ---- 规范要求:寄存器配置在 POWER DOWN 模式下进行 ----
     * CE=0 + PWR_UP=0 → POWER DOWN 模式,此阶段写入所有配置寄存器。
     * 配置完成后由 nrf24_set_rx_mode / nrf24_set_tx_mode 拉高 PWR_UP。
     * 注:init 在任务使用模组前完成(由事件组保证),此处不必加锁。
     */
    CE_LOW(mod);
    /* CONFIG: PWR_UP=0, EN_CRC=1, CRCO=1(2字节CRC), PRIM_RX=0 */
    nrf_write_reg(mod, NRF_REG_CONFIG, NRF_CONFIG_EN_CRC | NRF_CONFIG_CRCO);

    /* 禁用 Auto ACK(应用层处理ACK),开启 Pipe1 接收 */
    nrf_write_reg(mod, NRF_REG_EN_AA, 0x00);
    nrf_write_reg(mod, NRF_REG_EN_RXADDR, 0x02);

    /* 5字节地址宽度 */
    nrf_write_reg(mod, NRF_REG_SETUP_AW, 0x03);

    /* 禁用重发(Auto ACK已关闭) */
    nrf_write_reg(mod, NRF_REG_SETUP_RETR, 0x00);

    /* 射频参数(信道/速率/功率) */
    nrf24_config_rf(mod);

    /* 固定载荷长度(禁用动态载荷) */
    nrf_write_reg(mod, NRF_REG_FEATURE, 0x00);
    nrf_write_reg(mod, NRF_REG_DYNPD, 0x00);

    /* Pipe1 接收地址(本机地址) */
    nrf_write_buf(mod, NRF_REG_RX_ADDR_P1, mod->rx_addr, 5);
    nrf_write_reg(mod, NRF_REG_RX_PW_P1, mod->payload_len);

    /* TX 地址(发送目标地址) */
    nrf_write_buf(mod, NRF_REG_TX_ADDR, mod->tx_addr, 5);

    /* 验证地址配置 */
    {
        uint8_t addr[5];
        nrf_read_buf(mod, NRF_REG_RX_ADDR_P1, addr, 5);
        ESP_LOGI(TAG, "%s: RX_ADDR_P1: %c%c%c%c%c", mod->name, addr[0], addr[1], addr[2], addr[3], addr[4]);
        nrf_read_buf(mod, NRF_REG_TX_ADDR, addr, 5);
        ESP_LOGI(TAG, "%s: TX_ADDR: %c%c%c%c%c", mod->name, addr[0], addr[1], addr[2], addr[3], addr[4]);
    }

    /* 清空 FIFO */
    nrf_send_cmd(mod, NRF_CMD_FLUSH_TX);
    nrf_send_cmd(mod, NRF_CMD_FLUSH_RX);

    /* 清除中断标志(否则 IRQ 可能保持低电平) */
    nrf_write_reg(mod, NRF_REG_STATUS,
                  NRF_STATUS_RX_DR | NRF_STATUS_TX_DS | NRF_STATUS_MAX_RT);

    mod->is_rx_mode = false;

    uint8_t status = nrf_send_cmd(mod, NRF_CMD_NOP);
    ESP_LOGI(TAG, "%s: Init done, STATUS=0x%02X", mod->name, status);

    return ESP_OK;
}

esp_err_t nrf24_set_rx_mode(nrf24_module_t *mod)
{
    xSemaphoreTake(mod->lock, portMAX_DELAY);
    enter_rx_unlocked(mod, true); // 初次进入:清FIFO+全部中断标志
    xSemaphoreGive(mod->lock);

    ESP_LOGD(TAG, "%s: Enter RX mode (CE=1, LNA ON)", mod->name);
    return ESP_OK;
}

esp_err_t nrf24_set_tx_mode(nrf24_module_t *mod)
{
    xSemaphoreTake(mod->lock, portMAX_DELAY);
    enter_tx_unlocked(mod);
    nrf_send_cmd(mod, NRF_CMD_FLUSH_TX);
    xSemaphoreGive(mod->lock);

    ESP_LOGD(TAG, "%s: Enter TX mode (CE=0, waiting for send)", mod->name);
    return ESP_OK;
}

/**
 * 【核心修复】发送一包并等待芯片确认发射完成(TX_DS)。
 *
 * 原实现:CE脉冲后立即清标志+FLUSH_TX+切回RX,包在发射途中被中止,
 * 对端永远收不到(日志表现:双方 STATUS=0x0E, FIFO=0x11)。
 *
 * 现实现:CE脉冲后轮询STATUS等TX_DS(2Mbps典型~300µs,超时5ms);
 * 成功后载荷已自动移出TX FIFO,无需FLUSH;仅失败时FLUSH_TX防FIFO残留。
 */
esp_err_t nrf24_send_packet(nrf24_module_t *mod, const uint8_t *data, uint8_t len)
{
    if (mod == NULL || data == NULL || len == 0)
    {
        return ESP_ERR_INVALID_ARG;
    }
    if (len > NRF_MAX_PAYLOAD || len > mod->payload_len)
    {
        ESP_LOGW(TAG, "%s: Packet too long %d > %d", mod->name, len, mod->payload_len);
        return ESP_ERR_INVALID_SIZE;
    }

    xSemaphoreTake(mod->lock, portMAX_DELAY);

    bool was_rx = mod->is_rx_mode;
    if (was_rx)
    {
        enter_tx_unlocked(mod);
    }

    /* 写入载荷(不足固定长度补0) */
    uint8_t tx[33];
    uint8_t rx[33];
    tx[0] = NRF_CMD_W_TX_PAYLOAD;
    memcpy(&tx[1], data, len);
    if (len < mod->payload_len)
    {
        memset(&tx[1 + len], 0, mod->payload_len - len);
    }
    spi_xfer(mod, tx, rx, mod->payload_len + 1);

    /* CE 脉冲 ≥10µs 触发单包发送 */
    CE_HIGH(mod);
    esp_rom_delay_us(15);
    CE_LOW(mod);

    /* 等待发射完成:130µs TX settling + 空中时间(2Mbps约160µs) */
    uint8_t status = 0;
    uint32_t waited_us = 0;
    while (waited_us < NRF_TX_DS_TIMEOUT_US)
    {
        status = nrf_send_cmd(mod, NRF_CMD_NOP);
        if (status & (NRF_STATUS_TX_DS | NRF_STATUS_MAX_RT))
        {
            break;
        }
        esp_rom_delay_us(NRF_TX_DS_POLL_STEP_US);
        waited_us += NRF_TX_DS_POLL_STEP_US;
    }

    /* 【IRQ接线自检】TX_DS已置位且尚未清除 → IRQ引脚此刻物理上必须为低。
     * 读到高电平 = IRQ线未接通/接错脚(前5次发送各校验一次) */
    if ((status & NRF_STATUS_TX_DS) && mod->irq_wire_chk < 5)
    {
        mod->irq_wire_chk++;
        if (gpio_get_level(mod->irq_pin) != 0)
        {
            ESP_LOGW(TAG, "%s: TX_DS set but IRQ pin(GPIO%d)=HIGH -> IRQ line NOT connected!",
                     mod->name, mod->irq_pin);
        }
        else
        {
            ESP_LOGI(TAG, "%s: IRQ wiring OK (pin low while TX_DS set) [%d/5]",
                     mod->name, mod->irq_wire_chk);
        }
    }

    /* 清发送侧中断标志(写1清零,解除IRQ) */
    nrf_write_reg(mod, NRF_REG_STATUS, NRF_STATUS_TX_DS | NRF_STATUS_MAX_RT);

    esp_err_t err;
    if (status & NRF_STATUS_TX_DS)
    {
        err = ESP_OK;
        ESP_LOGD(TAG, "%s: TX done in ~%luus, STATUS=0x%02X",
                 mod->name, (unsigned long)waited_us, status);
    }
    else
    {
        /* 超时(或AA意外开启时MAX_RT):冲掉残留载荷,避免FIFO堆积 */
        nrf_send_cmd(mod, NRF_CMD_FLUSH_TX);
        err = ESP_ERR_TIMEOUT;
        ESP_LOGW(TAG, "%s: TX_DS timeout, STATUS=0x%02X", mod->name, status);
    }

    if (was_rx)
    {
        /* 回接收模式:不清RX FIFO/RX_DR,保留发送期间无法处理的状态 */
        enter_rx_unlocked(mod, false);
    }

    xSemaphoreGive(mod->lock);
    return err;
}

esp_err_t nrf24_read_packet(nrf24_module_t *mod, uint8_t *buf, uint8_t *len)
{
    xSemaphoreTake(mod->lock, portMAX_DELAY);

    if (rx_fifo_empty_unlocked(mod))
    {
        xSemaphoreGive(mod->lock);
        return ESP_ERR_NOT_FOUND;
    }

    uint8_t tx[33];
    uint8_t rx[33];
    tx[0] = NRF_CMD_R_RX_PAYLOAD;
    memset(&tx[1], 0xFF, mod->payload_len);
    spi_xfer(mod, tx, rx, mod->payload_len + 1);

    memcpy(buf, &rx[1], mod->payload_len);
    if (len)
        *len = mod->payload_len;

    /* 清RX中断标志 */
    nrf_write_reg(mod, NRF_REG_STATUS, NRF_STATUS_RX_DR);

    xSemaphoreGive(mod->lock);
    return ESP_OK;
}

void nrf24_set_rx_callback(nrf24_module_t *mod, nrf24_rx_callback_t cb)
{
    mod->rx_callback = cb;
}

void nrf24_clear_irq(nrf24_module_t *mod, uint8_t mask)
{
    xSemaphoreTake(mod->lock, portMAX_DELAY);
    nrf_write_reg(mod, NRF_REG_STATUS, mask);
    xSemaphoreGive(mod->lock);
}

/* ===================== 诊断工具 ===================== */

void nrf24_dump_regs(nrf24_module_t *mod)
{
    uint8_t p1[5], txa[5];

    xSemaphoreTake(mod->lock, portMAX_DELAY);
    uint8_t config = nrf_read_reg(mod, NRF_REG_CONFIG);
    uint8_t en_aa = nrf_read_reg(mod, NRF_REG_EN_AA);
    uint8_t en_rx = nrf_read_reg(mod, NRF_REG_EN_RXADDR);
    uint8_t aw = nrf_read_reg(mod, NRF_REG_SETUP_AW);
    uint8_t retr = nrf_read_reg(mod, NRF_REG_SETUP_RETR);
    uint8_t ch = nrf_read_reg(mod, NRF_REG_RF_CH);
    uint8_t rf = nrf_read_reg(mod, NRF_REG_RF_SETUP);
    uint8_t status = nrf_send_cmd(mod, NRF_CMD_NOP);
    uint8_t fifo = nrf_read_reg(mod, NRF_REG_FIFO_STATUS);
    uint8_t pw1 = nrf_read_reg(mod, NRF_REG_RX_PW_P1);
    uint8_t feat = nrf_read_reg(mod, NRF_REG_FEATURE);
    uint8_t dynpd = nrf_read_reg(mod, NRF_REG_DYNPD);
    nrf_read_buf(mod, NRF_REG_RX_ADDR_P1, p1, 5);
    nrf_read_buf(mod, NRF_REG_TX_ADDR, txa, 5);
    xSemaphoreGive(mod->lock);

    ESP_LOGI(TAG, "%s ==== REG DUMP ====", mod->name);
    ESP_LOGI(TAG, "%s CONFIG=0x%02X EN_AA=0x%02X EN_RXADDR=0x%02X AW=0x%02X RETR=0x%02X",
             mod->name, config, en_aa, en_rx, aw, retr);
    ESP_LOGI(TAG, "%s RF_CH=%d RF_SETUP=0x%02X STATUS=0x%02X FIFO=0x%02X",
             mod->name, ch, rf, status, fifo);
    ESP_LOGI(TAG, "%s RX_PW_P1=%d FEATURE=0x%02X DYNPD=0x%02X",
             mod->name, pw1, feat, dynpd);
    ESP_LOGI(TAG, "%s RX_ADDR_P1=%02X %02X %02X %02X %02X (%c%c%c%c%c)",
             mod->name, p1[0], p1[1], p1[2], p1[3], p1[4],
             p1[0], p1[1], p1[2], p1[3], p1[4]);
    ESP_LOGI(TAG, "%s TX_ADDR   =%02X %02X %02X %02X %02X (%c%c%c%c%c)",
             mod->name, txa[0], txa[1], txa[2], txa[3], txa[4],
             txa[0], txa[1], txa[2], txa[3], txa[4]);
}

uint8_t nrf24_carrier_detect(nrf24_module_t *mod)
{
    xSemaphoreTake(mod->lock, portMAX_DELAY);
    uint8_t cd = nrf_read_reg(mod, NRF_REG_CD) & 0x01;
    xSemaphoreGive(mod->lock);
    return cd;
}

void nrf24_carrier_test_start(nrf24_module_t *mod)
{
    xSemaphoreTake(mod->lock, portMAX_DELAY);
    CE_LOW(mod);
    nrf_write_reg(mod, NRF_REG_CONFIG,
                  NRF_CONFIG_EN_CRC | NRF_CONFIG_CRCO | NRF_CONFIG_PWR_UP);
    power_up_wait_unlocked(mod);
    uint8_t rf = nrf_read_reg(mod, NRF_REG_RF_SETUP);
    nrf_write_reg(mod, NRF_REG_RF_SETUP,
                  rf | NRF_RF_SETUP_CONT_WAVE | NRF_RF_SETUP_PLL_LOCK);
    CE_HIGH(mod);
    mod->is_rx_mode = false;
    xSemaphoreGive(mod->lock);
    ESP_LOGW(TAG, "%s: === CONSTANT CARRIER ON, ch=%d (TEST ONLY) ===",
             mod->name, mod->channel);
}

void nrf24_carrier_test_stop(nrf24_module_t *mod)
{
    xSemaphoreTake(mod->lock, portMAX_DELAY);
    CE_LOW(mod);
    uint8_t rf = nrf_read_reg(mod, NRF_REG_RF_SETUP);
    nrf_write_reg(mod, NRF_REG_RF_SETUP,
                  rf & (uint8_t)~(NRF_RF_SETUP_CONT_WAVE | NRF_RF_SETUP_PLL_LOCK));
    xSemaphoreGive(mod->lock);
    ESP_LOGW(TAG, "%s: carrier test off", mod->name);
}