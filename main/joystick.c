#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "joystick.h"

/*
 * ============================================================
 *  摇杆模拟桩:X/Y输出缓慢变化的圆形轨迹,按键每8次翻转一次,
 *  接收端看到连续变化的数值即证明链路+协议+解析全部正常。
 *
 *  接入真实摇杆时,将本文件替换为ADC实现:
 *    joystick_init():  配置ADC通道(如GPIO1/GPIO2,ADC1_CH0/CH1)
 *    joystick_read():  adc_oneshot_read → 减去中位校准值 → 映射到±1000
 *  头文件接口保持不变,上层零改动。
 * ============================================================
 */

static const char *TAG = "JOYSTICK";
static uint32_t s_tick = 0;

void joystick_init(void)
{
    ESP_LOGI(TAG, "Joystick init (SIMULATED stub, replace with ADC for real hardware)");
    s_tick = 0;
}

bool joystick_read(joystick_state_t *state)
{
    if (state == NULL)
    {
        return false;
    }

    // 仿真:围绕中心画圆,幅度800,周期16个采样点
    float angle = (float)(s_tick % 16) / 16.0f * 2.0f * (float)M_PI;
    state->x = (int16_t)(800.0f * cosf(angle));
    state->y = (int16_t)(800.0f * sinf(angle));
    state->buttons = (s_tick / 8) & 0x01; // 按键1周期性按下/抬起

    s_tick++;
    return true;
}