/**
  ******************************************************************************
  * @file    pwm.c
  * @brief   将 SBUS 归一化通道映射为 TIM2 两路 50Hz PWM
  *
  * 映射（板丝印 TIM2 脚）：
  *   右操纵杆：channels_norm[0] → TIM2_CH1 / PA0
  *   左操纵杆：channels_norm[3] → TIM2_CH3 / PA2
  *
 * 推进器脉宽（50Hz，高电平）：
 *   -1 → 1.0ms 反转满舵，0 → 1.5ms 停转，+1 → 2.0ms 正转满舵
 * 通道8（实测约 -0.966 / +0.985）：
 *   >0 手动，PWM 跟随 channels_norm[0]、[3]
 *   ≤0 暂不动作，两路停在 1.5ms；程序控制以后改 PWM_ApplyProgram()
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
    float pulse;

    if (norm > 1.0f) {
        norm = 1.0f;
    }
    if (norm < -1.0f) {
        norm = -1.0f;
    }

    /* 1MHz 计数时 CCR 就是高电平微秒数：1500 + norm*500 */
    pulse = (float)PWM_PULSE_NEUTRAL_US + norm * (float)PWM_PULSE_HALF_SPAN_US;
    if (pulse >= 0.0f) {
        pulse += 0.5f;
    } else {
        pulse -= 0.5f;
    }
    if (pulse < (float)PWM_PULSE_MIN_US) {
        return PWM_PULSE_MIN_US;
    }
    if (pulse > (float)PWM_PULSE_MAX_US) {
        return PWM_PULSE_MAX_US;
    }
    return (uint32_t)pulse;
}

static void PWM_WriteCompare(uint32_t ch, uint32_t pulse)
{
    __HAL_TIM_SET_COMPARE(&htim2, ch, pulse);
}

/* 通道8实测正档约 +0.985。大于 0 视为手动遥控。 */
static uint8_t PWM_IsManual(float mode_norm)
{
    return (mode_norm > 0.0f) ? 1U : 0U;
}

/* 通道8为负（实测约 -0.966）时走这里。当前只停转，后续改为程序给出的脉宽。 */
static void PWM_ApplyProgram(uint32_t *pulse0, uint32_t *pulse1)
{
    *pulse0 = PWM_PULSE_NEUTRAL_US;
    *pulse1 = PWM_PULSE_NEUTRAL_US;
}

void PWM_Init(void)
{
    PWM_EnableDwt();

    pwm_sync.pulse[0] = PWM_PULSE_NEUTRAL_US;
    pwm_sync.pulse[1] = PWM_PULSE_NEUTRAL_US;

    if (HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3) != HAL_OK) {
        Error_Handler();
    }

    /* 预装载 CCR，脉宽在周期边界更新。上电先停转（1.5ms） */
    __HAL_TIM_ENABLE_OCxPRELOAD(&htim2, TIM_CHANNEL_1);
    __HAL_TIM_ENABLE_OCxPRELOAD(&htim2, TIM_CHANNEL_3);

    PWM_WriteCompare(TIM_CHANNEL_1, PWM_PULSE_NEUTRAL_US);
    PWM_WriteCompare(TIM_CHANNEL_3, PWM_PULSE_NEUTRAL_US);
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

    pwm_sync.manual = PWM_IsManual(sbus_data.channels_norm[8]);
    if (pwm_sync.manual != 0U) {
        pulse0 = PWM_NormToPulse(sbus_data.channels_norm[0]);
        pulse1 = PWM_NormToPulse(sbus_data.channels_norm[3]);
    } else {
        PWM_ApplyProgram(&pulse0, &pulse1);
    }

    PWM_WriteCompare(TIM_CHANNEL_1, pulse0);
    PWM_WriteCompare(TIM_CHANNEL_3, pulse1);

    pwm_sync.pulse[0] = pulse0;
    pwm_sync.pulse[1] = pulse1;

    pwm_sync.pwm_out_tick_ms = HAL_GetTick();
    pwm_sync.pwm_out_us = PWM_NowUs();
    pwm_sync.dt_us = (int32_t)pwm_sync.pwm_out_us - (int32_t)pwm_sync.sbus_rx_us;
}
