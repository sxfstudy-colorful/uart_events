#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_log.h"
#include "board_config.h"
#include "joystick.h"

/*
 * ============================================================
 *  PS2双轴摇杆 ADC 实现(替换原模拟桩,对外接口不变)
 *
 *  硬件:URX/URY 模拟量(3.3V供电,中位≈1.65V,满偏0~3.3V)
 *        Z 数字按键(按下接地,需内部上拉)
 *
 *  处理链:多重采样均值 → 校准转mV → 减上电自校准中位
 *          → 死区 → 按正负两侧独立量程归一化到±1000 → 可选反向
 *
 *  为什么正负两侧独立归一化:中位不在量程正中间时
 *  (电位器个体差异),两侧行程不等长,统一比例会导致
 *  一侧推不满±1000、另一侧提前饱和。
 * ============================================================
 */

static const char *TAG = "JOYSTICK";

typedef struct
{
    adc_channel_t channel;
    adc_cali_handle_t cali; // NULL=校准方案不可用,退化为线性估算
    int center_mv;          // 上电自校准的中位电压
    int span_neg_mv;        // 负方向可用行程(center→0)
    int span_pos_mv;        // 正方向可用行程(center→满量程)
} axis_ctx_t;

static adc_oneshot_unit_handle_t s_adc_unit = NULL;
static axis_ctx_t s_axis_x;
static axis_ctx_t s_axis_y;
static bool s_initialized = false;

// 12dB衰减下ADC标称满量程(mV),校准不可用时的线性估算基准
#define ADC_FULL_SCALE_MV 3300
#define ADC_RAW_MAX 4095

/* ===================== 内部函数 ===================== */

/**
 * @brief 读取单轴电压(多重采样均值,mV)
 * @return 电压mV,失败返回-1
 */
static int axis_read_mv(axis_ctx_t *ax)
{
    int64_t sum_raw = 0;
    int raw;
    for (int i = 0; i < JOYSTICK_ADC_SAMPLES; i++)
    {
        if (adc_oneshot_read(s_adc_unit, ax->channel, &raw) != ESP_OK)
        {
            return -1;
        }
        sum_raw += raw;
    }
    int avg_raw = (int)(sum_raw / JOYSTICK_ADC_SAMPLES);

    int mv;
    if (ax->cali != NULL &&
        adc_cali_raw_to_voltage(ax->cali, avg_raw, &mv) == ESP_OK)
    {
        return mv;
    }
    // 校准方案不可用:线性估算
    return avg_raw * ADC_FULL_SCALE_MV / ADC_RAW_MAX;
}

/**
 * @brief 单轴初始化:配置通道 + 创建校准 + 中位自校准
 */
static bool axis_init(axis_ctx_t *ax, gpio_num_t gpio, const char *axis_name)
{
    // GPIO → ADC单元/通道
    adc_unit_t unit;
    if (adc_oneshot_io_to_channel(gpio, &unit, &ax->channel) != ESP_OK ||
        unit != ADC_UNIT_1)
    {
        ESP_LOGE(TAG, "%s: GPIO%d is not an ADC1 pin!", axis_name, gpio);
        return false;
    }

    adc_oneshot_chan_cfg_t ch_cfg = {
        .atten = ADC_ATTEN_DB_12, // 量程约0~3.3V,匹配摇杆3.3V供电满偏
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    if (adc_oneshot_config_channel(s_adc_unit, ax->channel, &ch_cfg) != ESP_OK)
    {
        ESP_LOGE(TAG, "%s: config channel failed", axis_name);
        return false;
    }

    // 校准方案(S3为曲线拟合);失败不致命,退化为线性估算
    adc_cali_curve_fitting_config_t cali_cfg = {
        .unit_id = ADC_UNIT_1,
        .chan = ax->channel,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    if (adc_cali_create_scheme_curve_fitting(&cali_cfg, &ax->cali) != ESP_OK)
    {
        ax->cali = NULL;
        ESP_LOGW(TAG, "%s: cali scheme unavailable, use linear estimate", axis_name);
    }

    // 中位自校准:采JOYSTICK_CAL_SAMPLES次平均(要求此时摇杆处于松开状态)
    int64_t sum = 0;
    int valid = 0;
    for (int i = 0; i < JOYSTICK_CAL_SAMPLES; i++)
    {
        int mv = axis_read_mv(ax);
        if (mv >= 0)
        {
            sum += mv;
            valid++;
        }
        vTaskDelay(pdMS_TO_TICKS(2));
    }
    if (valid == 0)
    {
        ESP_LOGE(TAG, "%s: center calibration failed (no valid samples)", axis_name);
        return false;
    }
    ax->center_mv = (int)(sum / valid);
    ax->span_neg_mv = ax->center_mv;                     // 中位→0V
    ax->span_pos_mv = ADC_FULL_SCALE_MV - ax->center_mv; // 中位→满量程

    // 合理性检查:3.3V供电中位理论1650mV,偏离过大提示接线/供电问题
    if (ax->center_mv < 1000 || ax->center_mv > 2300)
    {
        ESP_LOGW(TAG, "%s: center=%dmV deviates from ~1650mV, check wiring/power "
                      "(and keep stick released during boot)",
                 axis_name, ax->center_mv);
    }
    ESP_LOGI(TAG, "%s: GPIO%d ch%d center=%dmV span(-%d/+%d)mV",
             axis_name, gpio, ax->channel, ax->center_mv,
             ax->span_neg_mv, ax->span_pos_mv);
    return true;
}

/**
 * @brief mV → 归一化值(-1000~+1000,含死区,正负两侧独立量程)
 */
static int16_t axis_normalize(const axis_ctx_t *ax, int mv)
{
    int delta = mv - ax->center_mv;
    int val;
    if (delta >= 0)
    {
        val = (ax->span_pos_mv > 0) ? (delta * 1000 / ax->span_pos_mv) : 0;
    }
    else
    {
        val = (ax->span_neg_mv > 0) ? (delta * 1000 / ax->span_neg_mv) : 0;
    }

    // 死区
    if (abs(val) < JOYSTICK_DEADZONE)
    {
        return 0;
    }
    // 限幅
    if (val > 1000)
        val = 1000;
    if (val < -1000)
        val = -1000;
    return (int16_t)val;
}

/* ===================== 对外接口(与原桩完全一致) ===================== */

void joystick_init(void)
{
    if (s_initialized)
    {
        return;
    }

    // ADC1单元
    adc_oneshot_unit_init_cfg_t unit_cfg = {
        .unit_id = ADC_UNIT_1,
    };
    esp_err_t ret = adc_oneshot_new_unit(&unit_cfg, &s_adc_unit);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "ADC unit init failed: %d", ret);
        return;
    }

    // 两个模拟轴
    bool x_ok = axis_init(&s_axis_x, JOYSTICK_X_GPIO, "X-axis");
    bool y_ok = axis_init(&s_axis_y, JOYSTICK_Y_GPIO, "Y-axis");

    // Z按键:数字输入+内部上拉(模块SW按下接地)
    gpio_config_t btn_cfg = {
        .pin_bit_mask = 1ULL << JOYSTICK_Z_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&btn_cfg);

    s_initialized = x_ok && y_ok;
    ESP_LOGI(TAG, "Joystick init %s (ADC real hardware)",
             s_initialized ? "OK" : "FAILED");
}

bool joystick_read(joystick_state_t *state)
{
    if (state == NULL || !s_initialized)
    {
        return false;
    }

    int x_mv = axis_read_mv(&s_axis_x);
    int y_mv = axis_read_mv(&s_axis_y);
    if (x_mv < 0 || y_mv < 0)
    {
        ESP_LOGW(TAG, "ADC read failed");
        return false;
    }

    int16_t x = axis_normalize(&s_axis_x, x_mv);
    int16_t y = axis_normalize(&s_axis_y, y_mv);

#if JOYSTICK_INVERT_X
    x = -x;
#endif
#if JOYSTICK_INVERT_Y
    y = -y;
#endif

    state->x = x;
    state->y = y;
    // 按下=低电平 → bit0置1
    state->buttons = (gpio_get_level(JOYSTICK_Z_GPIO) == 0) ? 0x01 : 0x00;

    ESP_LOGD(TAG, "raw X=%dmV Y=%dmV → norm X=%d Y=%d BTN=0x%02X",
             x_mv, y_mv, state->x, state->y, state->buttons);
    return true;
}