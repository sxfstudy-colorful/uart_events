#ifndef BOARD_CONFIG_H
#define BOARD_CONFIG_H

#include "driver/uart.h"
#include "driver/gpio.h"

/*
 * ============================================================
 *  板级配置:整个工程唯一需要按部署形态修改的文件
 * ============================================================
 */

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

/* ---------- 接收模组引脚(HAS_RX_MODULE=1 时有效) ---------- */
#define LORA_RX_UART UART_NUM_1
#define LORA_RX_TX_PIN GPIO_NUM_5 // ESP TX → 模组 RXD(引脚7)
#define LORA_RX_RX_PIN GPIO_NUM_6 // ESP RX → 模组 TXD(引脚8)
#define LORA_RX_AUX_PIN GPIO_NUM_7

/* ---------- 发送模组引脚(HAS_TX_MODULE=1 时有效) ---------- */
#define LORA_TX_UART UART_NUM_2
#define LORA_TX_TX_PIN GPIO_NUM_15 // ESP TX → 模组 RXD(引脚7)
#define LORA_TX_RX_PIN GPIO_NUM_16 // ESP RX → 模组 TXD(引脚8)
#define LORA_TX_AUX_PIN GPIO_NUM_17

/* ---------- LoRa 配置策略 ----------
 * 模组参数(MODE/LEVEL/CHANNEL/MAC)掉电保存在模组内部,无需每次上电重写。
 *   ALWAYS      每次上电全量写入(最保险,启动最慢,消耗模组flash寿命)
 *   CHECK_FIRST 先回读校验,参数不符才写入(推荐:正常启动只多一次AT会话)
 *   SKIP        完全跳过(模组已配置好且不会被外部改动时,启动最快)
 */
#define LORA_CFG_ALWAYS 0
#define LORA_CFG_CHECK_FIRST 1
#define LORA_CFG_SKIP 2
#define LORA_CONFIG_POLICY LORA_CFG_CHECK_FIRST

/* ---------- LoRa 网络参数 ----------
 * 拓扑:遥控器(地址0002) --信道01--> 接收器(地址0001)
 * 注意:定点模式下"信道"决定接收方监听频点,收发双方的
 *      目标信道/自身信道必须按此表对齐。
 */
#define LORA_NET_LEVEL 3 // 空中速率等级(收发必须一致)
#define LORA_NET_MODE 1  // 1=定点传输

#define RECEIVER_ADDR_H 0x00 // 接收器地址 0001
#define RECEIVER_ADDR_L 0x01
#define RECEIVER_CHANNEL 0x01 // 接收器监听信道

#define CONTROLLER_ADDR_H 0x00 // 遥控器地址 0002
#define CONTROLLER_ADDR_L 0x02
#define CONTROLLER_CHANNEL 0x02 // 遥控器监听信道(为将来回传预留)

/* ---------- 摇杆硬件(HAS_TX_MODULE=1 时有效) ----------
 * PS2双轴摇杆模块:URX/URY模拟量 + Z数字按键
 * 【重要】模块必须3.3V供电!5V供电时满偏输出5V会超ADC引脚耐压。
 * ADC必须用ADC1(GPIO1~10),ADC2与WiFi冲突。
 */
#define JOYSTICK_X_GPIO GPIO_NUM_1 // URX → ADC1_CH0
#define JOYSTICK_Y_GPIO GPIO_NUM_2 // URY → ADC1_CH1
#define JOYSTICK_Z_GPIO GPIO_NUM_4 // Z按键,数字输入(内部上拉,按下=低)

#define JOYSTICK_ADC_SAMPLES 8  // 每次读取的多重采样次数(均值滤噪)
#define JOYSTICK_CAL_SAMPLES 32 // 上电中位自校准采样次数(此时勿碰摇杆)
#define JOYSTICK_DEADZONE 50    // 死区(归一化±1000量程下,即5%)
#define JOYSTICK_INVERT_X 0     // 方向反了改1
#define JOYSTICK_INVERT_Y 0

/* ---------- 业务参数 ---------- */
#define JOYSTICK_SEND_INTERVAL_MS 1000 // 摇杆状态发送周期(调试期1s,实际控制可降至50~100ms)

/* 链路ACK:接收器每收到一包摇杆数据回一个ACK,发送器据此在本地日志
 * 自证送达——两块板独立部署时无需同时盯两个串口即可检验链路。
 * 注意:开启后一个周期内要完成"发包+回ACK"两次空中传输,
 * AUX悬空(固定延时100ms/次)时发送周期建议 ≥500ms;
 * 接上AUX或调小TX_FIXED_DELAY_MS后可进一步压缩。 */
#define ENABLE_LINK_ACK 1

/* 失联感知:
 * 接收器侧——超过 LINK_TIMEOUT_MS 未收到摇杆包判定失联,触发failsafe
 *   (执行机构归零);取3~5倍发送周期,容忍偶发丢包不误判。
 * 遥控器侧——连续 ACK_MISS_LINK_DOWN 包未被ACK判定失联(main.c)。 */
#define LINK_TIMEOUT_MS (5 * JOYSTICK_SEND_INTERVAL_MS)
#define LINK_WATCHDOG_CHECK_MS 200 // 看门狗巡检周期

#endif // BOARD_CONFIG_H