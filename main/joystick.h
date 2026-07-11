#ifndef JOYSTICK_H
#define JOYSTICK_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C"
{
#endif

    /*
     * 摇杆采集抽象接口。
     * 当前实现为模拟桩(生成周期变化的仿真数据),用于无摇杆硬件时
     * 联调整条通信链路;接入真实摇杆(ADC)时只需重写 joystick.c,
     * 接口与上层调用完全不变。
     */

    typedef struct
    {
        int16_t x;       // 归一化 -1000 ~ +1000,中位0
        int16_t y;       // 归一化 -1000 ~ +1000,中位0
        uint8_t buttons; // 按键位图,bit0=按键1按下...
    } joystick_state_t;

    /** @brief 初始化摇杆硬件(桩实现为空操作) */
    void joystick_init(void);

    /**
     * @brief 读取当前摇杆状态
     * @return true=读取成功
     */
    bool joystick_read(joystick_state_t *state);

#ifdef __cplusplus
}
#endif

#endif // JOYSTICK_H