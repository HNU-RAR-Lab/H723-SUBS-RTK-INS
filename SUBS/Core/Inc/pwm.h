/**
  ******************************************************************************
  * @file    pwm.h
  * @brief   SBUS 两路 50Hz 推进器 PWM（脉宽 1～2ms）
  ******************************************************************************
  */

#ifndef __PWM_H
#define __PWM_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* TIM2 时钟 200MHz：PSC=199 → 1MHz（1计数=1us），ARR=19999 → 50Hz（20ms） */
#define PWM_TIM_PRESCALER       199U
#define PWM_TIM_PERIOD          19999U
#define PWM_PULSE_MIN_US        1000U   /* norm=-1，反转满舵 */
#define PWM_PULSE_NEUTRAL_US    1500U   /* norm= 0，停转 */
#define PWM_PULSE_MAX_US        2000U   /* norm=+1，正转满舵 */
#define PWM_PULSE_HALF_SPAN_US  500U    /* 中位到满舵的脉宽增量 */
/* 通道8实测约 -0.966 / +0.985，以 0 为界：正=手动，负=不跟随摇杆 */

typedef struct {
    volatile uint32_t sbus_rx_tick_ms;  /* SBUS 解帧时刻，HAL_GetTick() */
    volatile uint32_t pwm_out_tick_ms;  /* 写入 CCR 时刻，同一时基 */
    volatile uint32_t sbus_rx_us;       /* SBUS 解帧时刻，DWT 微秒 */
    volatile uint32_t pwm_out_us;       /* 写入 CCR 时刻，同一时基 */
    volatile uint32_t pulse[2];         /* PA0/PA2 高电平脉宽，单位 us */
    volatile uint8_t  manual;           /* 1：通道8为正，摇杆直接驱动 PWM */
    volatile int32_t  dt_us;            /* pwm_out_us - sbus_rx_us，正常约 0～数十 us */
} PWM_Sync_t;

void PWM_Init(void);
void PWM_SyncFromSbus(void);

extern PWM_Sync_t pwm_sync;

#ifdef __cplusplus
}
#endif

#endif /* __PWM_H */
