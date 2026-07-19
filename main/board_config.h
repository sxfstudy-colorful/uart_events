#ifndef BOARD_CONFIG_H
#define BOARD_CONFIG_H

#include "driver/uart.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"

/*
 * ============================================================
 *  板级配置:整个工程唯一需要按部署形态修改的文件
 * ============================================================
 */

/* ---------- 无线模组类型 ---------- */
#define RADIO_LORA_UART 0 // LoRa UART 模组 (DX-LR41-24T12S)
#define RADIO_NRF24_SPI 1 // nRF24L01+ SPI 模组 (E01-ML01DP5)
#define RADIO_TYPE RADIO_NRF24_SPI

/* ---------- 设备角色 ---------- */
#define ROLE_DUAL_DEBUG 0 // 同机调试:一块ESP32挂两个模组,自发自收
#define ROLE_CONTROLLER 1 // 遥控器:仅发送模组 + 摇杆
#define ROLE_RECEIVER 2   // 接收器:仅接收模组

// 【迁移到两块板时,各自只改这一行】
#define DEVICE_ROLE ROLE_CONTROLLER

/* 由角色派生的模块存在性,后续所有条件编译只看这两个宏 */
#if (DEVICE_ROLE == ROLE_DUAL_DEBUG)
#define HAS_TX_MODULE 1
#define HAS_RX_MODULE 1
#elif (DEVICE_ROLE == ROLE_CONTROLLER)
#define HAS_TX_MODULE 1
#define HAS_RX_MODULE 0
#elif (DEVICE_ROLE == ROLE_RECEIVER)
#define HAS_TX_MODULE 0
#define HAS_RX_MODULE 1
#else
#error "Invalid DEVICE_ROLE"
#endif

/* ---------- 硬件形态 ---------- */
// AUX引脚是否已接线:0=悬空(固定延时代替检测) 1=已接线
#define AUX_CONNECTED 0

#if (RADIO_TYPE == RADIO_LORA_UART)

/* ---------- LoRa UART 接收模组引脚(HAS_RX_MODULE=1 时有效) ---------- */
#define LORA_RX_UART UART_NUM_1
#define LORA_RX_TX_PIN GPIO_NUM_5
#define LORA_RX_RX_PIN GPIO_NUM_6
#define LORA_RX_AUX_PIN GPIO_NUM_7

/* ---------- LoRa UART 发送模组引脚(HAS_TX_MODULE=1 时有效) ---------- */
#define LORA_TX_UART UART_NUM_2
#define LORA_TX_TX_PIN GPIO_NUM_15
#define LORA_TX_RX_PIN GPIO_NUM_16
#define LORA_TX_AUX_PIN GPIO_NUM_17

/* ---------- LoRa 配置策略 ---------- */
#define LORA_CFG_ALWAYS 0
#define LORA_CFG_CHECK_FIRST 1
#define LORA_CFG_SKIP 2
#define LORA_CONFIG_POLICY LORA_CFG_CHECK_FIRST

/* ---------- LoRa 网络参数 ---------- */
#define LORA_NET_LEVEL 3
#define LORA_NET_MODE 1

#define RECEIVER_ADDR_H 0x00
#define RECEIVER_ADDR_L 0x01
#define RECEIVER_CHANNEL 0x01

#define CONTROLLER_ADDR_H 0x00
#define CONTROLLER_ADDR_L 0x02
#define CONTROLLER_CHANNEL 0x02

#elif (RADIO_TYPE == RADIO_NRF24_SPI)

/* ---------- nRF24 接收模组 (HAS_RX_MODULE=1) ----------
 * 独立SPI2控制器,接线清晰,不与TX模组共享总线
 */
#define NRF24_RX_SPI_HOST SPI2_HOST
#define NRF24_RX_SCK_PIN GPIO_NUM_12
#define NRF24_RX_MOSI_PIN GPIO_NUM_11
#define NRF24_RX_MISO_PIN GPIO_NUM_13
#define NRF24_RX_CE_PIN GPIO_NUM_5
#define NRF24_RX_CSN_PIN GPIO_NUM_6
#define NRF24_RX_IRQ_PIN GPIO_NUM_7

/* ---------- nRF24 发送模组 (HAS_TX_MODULE=1) ----------
 * 独立SPI3控制器,接线清晰,不与RX模组共享总线
 */
#define NRF24_TX_SPI_HOST SPI3_HOST
#define NRF24_TX_SCK_PIN GPIO_NUM_15
#define NRF24_TX_MOSI_PIN GPIO_NUM_16
#define NRF24_TX_MISO_PIN GPIO_NUM_17
#define NRF24_TX_CE_PIN GPIO_NUM_9
#define NRF24_TX_CSN_PIN GPIO_NUM_18
#define NRF24_TX_IRQ_PIN GPIO_NUM_8

/* ---------- nRF24 射频参数 (收发必须一致) ---------- */
#define NRF24_CHANNEL 60           // 信道 0~125 (2400+channel MHz)
#define NRF24_SPEED NRF24_SPEED_2M // 空中速率
#define NRF24_POWER NRF24_PWR_0DBM // 发射功率
#define NRF24_PAYLOAD_LEN 32       // 固定载荷长度 (1~32)

/* 接收地址 (Pipe1, 5字节) - 接收器监听地址 */
#define NRF24_RECEIVER_ADDR {'E', 'F', 'T', '0', '1'}
/* 发送目标地址 - 遥控器发往接收器 */
#define NRF24_CONTROLLER_ADDR {'E', 'F', 'T', '0', '2'}

#endif // RADIO_TYPE

/* ---------- 摇杆硬件(HAS_TX_MODULE=1 时有效) ----------
 * PS2双轴摇杆模块:URX/URY模拟量 + Z数字按键
 * 【重要】模块必须3.3V供电!5V供电时满偏输出5V会超ADC引脚耐压。
 * ADC必须用ADC1(GPIO1~10),ADC2与WiFi冲突。
 */
#define JOYSTICK_X_GPIO GPIO_NUM_1 // URX → ADC1_CH0
#define JOYSTICK_Y_GPIO GPIO_NUM_2 // URY → ADC1_CH1
#define JOYSTICK_Z_GPIO GPIO_NUM_4 // Z按键,数字输入(内部上拉,按下=低)

#define JOYSTICK_ADC_SAMPLES 8
#define JOYSTICK_CAL_SAMPLES 32
#define JOYSTICK_DEADZONE 50
#define JOYSTICK_INVERT_X 0
#define JOYSTICK_INVERT_Y 0

/* ---------- 业务参数 ---------- */
#define JOYSTICK_SEND_INTERVAL_MS 200 // 摇杆状态发送周期(调试期1s)

#define HEARTBEAT_INTERVAL_MS 2000 // 心跳包发送周期(ms)
#define HEARTBEAT_MISS_LINK_DOWN 3 // 连续多少次心跳无ACK判链路断开

#define LINK_TIMEOUT_MS (5 * JOYSTICK_SEND_INTERVAL_MS)
#define LINK_WATCHDOG_CHECK_MS 200

#endif // BOARD_CONFIG_H
