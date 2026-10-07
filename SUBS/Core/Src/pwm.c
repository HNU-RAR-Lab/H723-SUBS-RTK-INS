/**
  ******************************************************************************
  * @file    pwm.c
  * @brief   将 SBUS 归一化通道映射为 TIM2 两路 50Hz PWM
  *
  * 映射（板丝印 TIM2 脚）：
  *   右操纵杆：channels_norm[0] → TIM2_CH1 / PA0
  *   左操纵杆：channels_norm[3] → TIM2_CH3 / PA2
  *
  * 占空比：左满舵：-1 → 0%，居中：0 → 50%，右满舵：+1 → 100%
  * 频率：50Hz（周期 20ms）
  *
  * 时间戳：SBUS 解帧与 CCR 更新共用 HAL_GetTick 与 DWT 微秒计数，
  *         主循环在 Poll 之后立即 Sync，使两路时间戳对齐到同一帧。
  ******************************************************************************
  */

#include "pwm.h"
#include "sbus.h"
#include "main.h"

extern TIM_HandleTypeDef htim2;

PWM_Sync_t pwm_sync;

static uint32_t PWM_NowUs(void)
{
    const uint32_t cycles_per_us = SystemCoreClock / 1000000U;
    if (cycles_per_us == 0U) {
        return 0U;
    }
    return DWT->CYCCNT / cycles_per_us;
}

static void PWM_EnableDwt(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static uint32_t PWM_NormToPulse(float norm)
{
    const uint32_t counts = PWM_TIM_PERIOD + 1U;
    float duty;
    uint32_t pulse;

    if (norm > 1.0f) {
        norm = 1.0f;
    }
    if (norm < -1.0f) {
        norm = -1.0f;
    }

    /* duty = (norm + 1) / 2  →  [-1,0,+1] 对应 [0, 0.5, 1.0] */
    duty = (norm + 1.0f) * 0.5f;
    pulse = (uint32_t)(duty * (float)counts + 0.5f);
    if (pulse > counts) {
        pulse = counts;
    }
    return pulse;
}

static void PWM_WriteCompare(uint32_t ch, uint32_t pulse)
{
    __HAL_TIM_SET_COMPARE(&htim2, ch, pulse);
}

void PWM_Init(void)
{
    PWM_EnableDwt();

    pwm_sync.pulse[0] = PWM_PULSE_50_PERCENT;
    pwm_sync.pulse[1] = PWM_PULSE_50_PERCENT;

    if (HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3) != HAL_OK) {
        Error_Handler();
    }

    /* 预装载 CCR，占空比在周期边界更新，避免毛刺 */
    __HAL_TIM_ENABLE_OCxPRELOAD(&htim2, TIM_CHANNEL_1);
    __HAL_TIM_ENABLE_OCxPRELOAD(&htim2, TIM_CHANNEL_3);

    PWM_WriteCompare(TIM_CHANNEL_1, PWM_PULSE_50_PERCENT);
    PWM_WriteCompare(TIM_CHANNEL_3, PWM_PULSE_50_PERCENT);
}

void PWM_SyncFromSbus(void)
{
    static uint32_t last_frame_ok;
    uint32_t pulse0;
    uint32_t pulse1;

    if (sbus_data.updated == 0U) {
        return;
    }
    if (sbus_dbg.frame_ok_cnt == last_frame_ok) {
        return;
    }
    last_frame_ok = sbus_dbg.frame_ok_cnt;

    /* 先记下 SBUS 解帧时间戳，再立刻改 CCR，保证同一帧对齐 */
    pwm_sync.sbus_rx_tick_ms = sbus_data.rx_tick_ms;
    pwm_sync.sbus_rx_us = sbus_data.rx_us;

    pulse0 = PWM_NormToPulse(sbus_data.channels_norm[0]);
    pulse1 = PWM_NormToPulse(sbus_data.channels_norm[3]);

    PWM_WriteCompare(TIM_CHANNEL_1, pulse0);
    PWM_WriteCompare(TIM_CHANNEL_3, pulse1);

    pwm_sync.pulse[0] = pulse0;
    pwm_sync.pulse[1] = pulse1;

    pwm_sync.pwm_out_tick_ms = HAL_GetTick();
    pwm_sync.pwm_out_us = PWM_NowUs();
    pwm_sync.dt_us = (int32_t)pwm_sync.pwm_out_us - (int32_t)pwm_sync.sbus_rx_us;
}
