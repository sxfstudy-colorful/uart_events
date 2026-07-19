#ifndef NRF24L01_H
#define NRF24L01_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "freertos/semphr.h"

/*
 * ============================================================
 *  nRF24L01+ (E01-ML01DP5) 驱动
 *  SPI接口,PA+LNA,20dBm,2.4GHz
 *
 *  【修复记录 2026-07】
 *  1. nrf24_send_packet 原先在CE脉冲后立即FLUSH_TX并切回RX,
 *     包在发射途中被中止,接收端永远收不到。现改为等待TX_DS。
 *  2. 每模组增加互斥锁 lock:joystick_tx / heartbeat_tx /
 *     rx_poll 多任务并发操作同一模组的SPI与模式切换,必须串行化。
 *  3. 首次 PWR_UP 后等待 Tpd2stby(≤1.5ms) 再拉CE。
 * ============================================================
 */

/* ---------- 寄存器地址 ---------- */
#define NRF_REG_CONFIG 0x00
#define NRF_REG_EN_AA 0x01
#define NRF_REG_EN_RXADDR 0x02
#define NRF_REG_SETUP_AW 0x03
#define NRF_REG_SETUP_RETR 0x04
#define NRF_REG_RF_CH 0x05
#define NRF_REG_RF_SETUP 0x06
#define NRF_REG_STATUS 0x07
#define NRF_REG_OBSERVE_TX 0x08
#define NRF_REG_CD 0x09
#define NRF_REG_RX_ADDR_P0 0x0A
#define NRF_REG_RX_ADDR_P1 0x0B
#define NRF_REG_RX_ADDR_P2 0x0C
#define NRF_REG_RX_ADDR_P3 0x0D
#define NRF_REG_RX_ADDR_P4 0x0E
#define NRF_REG_RX_ADDR_P5 0x0F
#define NRF_REG_TX_ADDR 0x10
#define NRF_REG_RX_PW_P0 0x11
#define NRF_REG_RX_PW_P1 0x12
#define NRF_REG_RX_PW_P2 0x13
#define NRF_REG_RX_PW_P3 0x14
#define NRF_REG_RX_PW_P4 0x15
#define NRF_REG_RX_PW_P5 0x16
#define NRF_REG_FIFO_STATUS 0x17
#define NRF_REG_DYNPD 0x1C
#define NRF_REG_FEATURE 0x1D

/* ---------- 指令 ---------- */
#define NRF_CMD_R_REGISTER 0x00
#define NRF_CMD_W_REGISTER 0x20
#define NRF_CMD_R_RX_PAYLOAD 0x61
#define NRF_CMD_W_TX_PAYLOAD 0xA0
#define NRF_CMD_FLUSH_TX 0xE1
#define NRF_CMD_FLUSH_RX 0xE2
#define NRF_CMD_REUSE_TX_PL 0xE3
#define NRF_CMD_NOP 0xFF

/* ---------- CONFIG 寄存器位 ---------- */
#define NRF_CONFIG_MASK_RX_DR BIT(6)
#define NRF_CONFIG_MASK_TX_DS BIT(5)
#define NRF_CONFIG_MASK_MAX_RT BIT(4)
#define NRF_CONFIG_EN_CRC BIT(3)
#define NRF_CONFIG_CRCO BIT(2)
#define NRF_CONFIG_PWR_UP BIT(1)
#define NRF_CONFIG_PRIM_RX BIT(0)

/* ---------- STATUS 寄存器位 ---------- */
#define NRF_STATUS_RX_DR BIT(6)
#define NRF_STATUS_TX_DS BIT(5)
#define NRF_STATUS_MAX_RT BIT(4)
#define NRF_STATUS_RX_P_NO_MASK (BIT(3) | BIT(2) | BIT(1))
#define NRF_STATUS_TX_FULL BIT(0)

/* ---------- RF_SETUP ---------- */
#define NRF_RF_SETUP_CONT_WAVE BIT(7) // 恒定载波测试模式(诊断用)
#define NRF_RF_SETUP_RF_DR_HIGH BIT(3)
#define NRF_RF_SETUP_RF_DR_LOW BIT(5)
#define NRF_RF_SETUP_PLL_LOCK BIT(4)
#define NRF_RF_PWR_0DBM (BIT(2) | BIT(1))
#define NRF_RF_PWR_MIN6DBM BIT(2)
#define NRF_RF_PWR_MIN12DBM BIT(1)
#define NRF_RF_PWR_MIN18DBM 0

/* ---------- 速率 ---------- */
#define NRF_SPEED_250K (NRF_RF_SETUP_RF_DR_LOW)
#define NRF_SPEED_1M (0)
#define NRF_SPEED_2M (NRF_RF_SETUP_RF_DR_HIGH)

/* ---------- FIFO_STATUS ---------- */
#define NRF_FIFO_TX_REUSE BIT(6)
#define NRF_FIFO_TX_FULL BIT(5)
#define NRF_FIFO_TX_EMPTY BIT(4)
#define NRF_FIFO_RX_FULL BIT(1)
#define NRF_FIFO_RX_EMPTY BIT(0)

/* ---------- 最大载荷 ---------- */
#define NRF_MAX_PAYLOAD 32

/* ---------- 发送完成等待 ----------
 * 2Mbps: 130µs TX settling + ~160µs 空中时间(40字节含前导/地址/CRC)。
 * 250Kbps 最慢约 1.4ms,统一给 5ms 超时上限,轮询步长 20µs。
 */
#define NRF_TX_DS_POLL_STEP_US 20
#define NRF_TX_DS_TIMEOUT_US 5000

/* ---------- 数据速率可选值 ---------- */
typedef enum
{
    NRF24_SPEED_250K = 0,
    NRF24_SPEED_1M = 1,
    NRF24_SPEED_2M = 2,
} nrf24_speed_t;

/* ---------- 发射功率可选值 ---------- */
typedef enum
{
    NRF24_PWR_MIN18DBM = 0,
    NRF24_PWR_MIN12DBM = 1,
    NRF24_PWR_MIN6DBM = 2,
    NRF24_PWR_0DBM = 3,
} nrf24_power_t;

/* ---------- 接收回调类型 ---------- */
struct nrf24_module_s;
typedef void (*nrf24_rx_callback_t)(const struct nrf24_module_s *mod,
                                    const uint8_t *data, size_t len);

/* ---------- 模块配置 ---------- */
typedef struct nrf24_module_s
{
    const char *name;

    /* SPI硬件 */
    spi_host_device_t spi_host;
    gpio_num_t ce_pin;
    gpio_num_t csn_pin;
    gpio_num_t sck_pin;
    gpio_num_t mosi_pin;
    gpio_num_t miso_pin;
    gpio_num_t irq_pin;

    /* 射频参数 */
    uint8_t channel;     // 信道 0~125 (2400+channel MHz)
    nrf24_speed_t speed; // 空中速率
    nrf24_power_t power; // 发射功率
    uint8_t rx_addr[5];  // 本机接收地址(Pipe1)
    uint8_t tx_addr[5];  // 发送目标地址
    uint8_t payload_len; // 固定载荷长度(1~32)

    /* 运行时 */
    spi_device_handle_t spi_handle;
    bool is_rx_mode;
    bool powered_up;           // 【新增】是否已完成掉电→Standby-I上电等待
    uint8_t irq_wire_chk;      // 【新增】IRQ接线自检次数(前5次发送时校验)
    SemaphoreHandle_t lock;    // 【新增】模组级互斥锁(SPI+模式切换串行化)
    SemaphoreHandle_t irq_sem; // IRQ中断信号量(发送完成/接收就绪)
    nrf24_rx_callback_t rx_callback;
} nrf24_module_t;

/* ---------- 公共接口 ---------- */

esp_err_t nrf24_init(nrf24_module_t *mod);

esp_err_t nrf24_set_rx_mode(nrf24_module_t *mod);
esp_err_t nrf24_set_tx_mode(nrf24_module_t *mod);

/**
 * @brief 发送一包(阻塞直到TX_DS或超时)
 * @return ESP_OK=芯片确认已发射完成;ESP_ERR_TIMEOUT=未收到TX_DS
 * 内部自动:RX→TX切换、等待发送完成、切回RX。线程安全(内部加锁)。
 */
esp_err_t nrf24_send_packet(nrf24_module_t *mod, const uint8_t *data, uint8_t len);

esp_err_t nrf24_read_packet(nrf24_module_t *mod, uint8_t *buf, uint8_t *len);

void nrf24_set_rx_callback(nrf24_module_t *mod, nrf24_rx_callback_t cb);

/* 等待IRQ中断(低电平有效),中断触发后返回pdTRUE,超时返回pdFALSE */
BaseType_t nrf24_wait_irq(nrf24_module_t *mod, TickType_t timeout);

/* 清除STATUS中断标志(写1清零),解除IRQ低电平 */
void nrf24_clear_irq(nrf24_module_t *mod, uint8_t mask);

bool nrf24_tx_fifo_empty(nrf24_module_t *mod);
bool nrf24_rx_fifo_empty(nrf24_module_t *mod);

uint8_t nrf24_read_status(nrf24_module_t *mod);
uint8_t nrf24_read_reg(nrf24_module_t *mod, uint8_t reg);

/* ---------- 诊断工具(定位RF层故障用) ---------- */

/** @brief 打印全部关键寄存器与收发地址,用于两端配置比对 */
void nrf24_dump_regs(nrf24_module_t *mod);

/** @brief 读CD载波检测位:1=当前信道检测到载波(>-64dBm持续128µs) */
uint8_t nrf24_carrier_detect(nrf24_module_t *mod);

/**
 * @brief 开启恒定载波发射(CONT_WAVE+PLL_LOCK,CE=1),仅测试用!
 * 配合对端 nrf24_carrier_detect 验证RF通路。测完必须调 stop。
 */
void nrf24_carrier_test_start(nrf24_module_t *mod);
void nrf24_carrier_test_stop(nrf24_module_t *mod);

#endif