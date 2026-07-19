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
     * 当前实现:PS2双轴摇杆ADC采集(joystick.c),
     * URX/URY走ADC1多重采样+校准+中位自校准+死区,Z为数字按键。
     * 引脚与参数在 board_config.h 的摇杆硬件段配置。
     * 上层(main.c的joystick_tx_task)只依赖本接口,与实现解耦。
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