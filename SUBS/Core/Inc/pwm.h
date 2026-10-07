/**
  ******************************************************************************
  * @file    pwm.h
  * @brief   SBUS 两路 50Hz PWM 输出与收/发时间戳同步
  ******************************************************************************
  */

#ifndef __PWM_H
#define __PWM_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* TIM2 时钟 200MHz：PSC=199 → 1MHz 计数，ARR=19999 → 50Hz（20ms） */
#define PWM_TIM_PRESCALER     199U
#define PWM_TIM_PERIOD        19999U
#define PWM_PULSE_50_PERCENT  10000U

typedef struct {
    volatile uint32_t sbus_rx_tick_ms;  /* SBUS 解帧时刻，HAL_GetTick() */
    volatile uint32_t pwm_out_tick_ms;  /* 写入 CCR 时刻，同一时基 */
    volatile uint32_t sbus_rx_us;       /* SBUS 解帧时刻，DWT 微秒 */
    volatile uint32_t pwm_out_us;       /* 写入 CCR 时刻，同一时基 */
    volatile uint32_t pulse[2];         /* CH1/CH3 当前比较值，对应 SBUS[0]/[3] */
    volatile int32_t  dt_us;            /* pwm_out_us - sbus_rx_us，正常约 0～数十 us */
} PWM_Sync_t;

void PWM_Init(void);
void PWM_SyncFromSbus(void);

extern PWM_Sync_t pwm_sync;

#ifdef __cplusplus
}
#endif

#endif /* __PWM_H */
