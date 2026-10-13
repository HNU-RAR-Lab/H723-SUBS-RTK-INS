/**
  ******************************************************************************
  * @file    sbus.h
  * @brief   R9DS SBUS 协议解析（DM-MC02 板载 SUBS 已硬件反相，MCU 内帧头 0x0F）
  ******************************************************************************
  */

#ifndef __SBUS_H
#define __SBUS_H

#include <stdint.h>
#include "stm32h7xx_hal.h"

#define SBUS_FRAME_LEN     25
#define SBUS_HEADER        0x0FU  /* 板载硬件反相后，PD2 上标准 SBUS 帧头 */
#define SBUS_FOOTER        0x00U
#define SBUS_CHANNEL_NUM   16
#define SBUS_DMA_BUF_LEN   128

/* Futaba SBUS 典型脉宽编码范围，中位 992 对应 0，两端约 ±1 */
#define SBUS_CH_MIN        172U
#define SBUS_CH_CENTER     992U
#define SBUS_CH_MAX        1811U

typedef struct {
    volatile uint16_t channels[SBUS_CHANNEL_NUM];      /* 原始值 172～1811 */
    volatile float    channels_norm[SBUS_CHANNEL_NUM]; /* 归一化 -1.0～1.0 */
    volatile uint8_t  frame_lost;
    volatile uint8_t  failsafe;
    volatile uint8_t  updated;
    volatile uint32_t rx_tick_ms; /* 解帧成功时刻，与 PWM 共用 HAL_GetTick */
    volatile uint32_t rx_us;      /* 解帧成功时刻，与 PWM 共用 DWT 微秒 */
} SBUS_Data_t;

typedef struct {
    volatile uint32_t dma_poll_cnt;
    volatile uint32_t rx_event_cnt;
    volatile uint32_t frame_ok_cnt;
    volatile uint32_t frame_err_cnt;
    volatile uint32_t header_hit_cnt;
    volatile uint32_t uart_err_cnt;
    volatile uint32_t last_uart_err;
    volatile uint16_t last_dma_pos;
    volatile uint8_t  last_header;
    volatile uint8_t  last_footer;
    volatile uint8_t  rxinv_hw;       /* 应为 0：软件 RXINV 已关闭 */
    volatile uint8_t  raw[SBUS_FRAME_LEN];
} SBUS_Debug_t;

int  SBUS_ParseFrame(const uint8_t *buf, SBUS_Data_t *data);
void SBUS_FeedByte(uint8_t byte);
void SBUS_OnDmaIdle(const volatile uint8_t *buf, uint16_t buf_len, uint16_t dma_pos);
void SBUS_PollFromUart(UART_HandleTypeDef *huart, const volatile uint8_t *buf, uint16_t buf_len);
void Debug_Printf(const char *fmt, ...);
void Debug_Write(const uint8_t *data, uint16_t len);

extern SBUS_Data_t sbus_data;
extern SBUS_Debug_t sbus_dbg;
extern volatile uint8_t sbus_dma_buf[SBUS_DMA_BUF_LEN];

#endif /* __SBUS_H */
