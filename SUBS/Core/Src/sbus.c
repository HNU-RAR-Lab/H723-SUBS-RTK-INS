/**
  ******************************************************************************
  * @file    sbus.c
  * @brief   SBUS 组帧：0x0F…0x00（DM-MC02 板载硬件反相，软件 RXINV 关闭）
  ******************************************************************************
  */

#include "sbus.h"
#include "main.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

extern UART_HandleTypeDef huart10;

SBUS_Data_t sbus_data;
SBUS_Debug_t sbus_dbg;

static uint8_t  sbus_rx_frame[SBUS_FRAME_LEN];
static uint8_t  sbus_rx_index;
static uint16_t sbus_dma_last_pos;

static float SBUS_NormalizeChannel(uint16_t raw) //归一化处理
{
    const float half_span = (float)(SBUS_CH_MAX - SBUS_CH_MIN) * 0.5f;
    float norm;

    norm = ((float)raw - (float)SBUS_CH_CENTER) / half_span;
    if (norm > 1.0f) {
        return 1.0f;
    }
    if (norm < -1.0f) {
        return -1.0f;
    }
    return norm;
}

int SBUS_ParseFrame(const uint8_t *buf, SBUS_Data_t *data)
{
    uint8_t i;

    sbus_dbg.last_header = buf[0];
    sbus_dbg.last_footer = buf[SBUS_FRAME_LEN - 1U];
    for (i = 0U; i < SBUS_FRAME_LEN; i++) {
        sbus_dbg.raw[i] = buf[i];
    }

    if ((buf[0] != SBUS_HEADER) || (buf[SBUS_FRAME_LEN - 1U] != SBUS_FOOTER)) {
        sbus_dbg.frame_err_cnt++;
        return 0;
    }

    data->channels[0]  = (uint16_t)((buf[1]       | buf[2] << 8)                         & 0x07FF);
    data->channels[1]  = (uint16_t)((buf[2] >> 3  | buf[3] << 5)                         & 0x07FF);
    data->channels[2]  = (uint16_t)((buf[3] >> 6  | buf[4] << 2 | buf[5] << 10)          & 0x07FF);
    data->channels[3]  = (uint16_t)((buf[5] >> 1  | buf[6] << 7)                         & 0x07FF);
    data->channels[4]  = (uint16_t)((buf[6] >> 4  | buf[7] << 4)                         & 0x07FF);
    data->channels[5]  = (uint16_t)((buf[7] >> 7  | buf[8] << 1 | buf[9] << 9)           & 0x07FF);
    data->channels[6]  = (uint16_t)((buf[9] >> 2  | buf[10] << 6)                        & 0x07FF);
    data->channels[7]  = (uint16_t)((buf[10] >> 5 | buf[11] << 3)                       & 0x07FF);
    data->channels[8]  = (uint16_t)((buf[12]      | buf[13] << 8)                        & 0x07FF);
    data->channels[9]  = (uint16_t)((buf[13] >> 3 | buf[14] << 5)                       & 0x07FF);
    data->channels[10] = (uint16_t)((buf[14] >> 6 | buf[15] << 2 | buf[16] << 10)      & 0x07FF);
    data->channels[11] = (uint16_t)((buf[16] >> 1 | buf[17] << 7)                       & 0x07FF);
    data->channels[12] = (uint16_t)((buf[17] >> 4 | buf[18] << 4)                       & 0x07FF);
    data->channels[13] = (uint16_t)((buf[18] >> 7 | buf[19] << 1 | buf[20] << 9)       & 0x07FF);
    data->channels[14] = (uint16_t)((buf[20] >> 2 | buf[21] << 6)                       & 0x07FF);
    data->channels[15] = (uint16_t)((buf[21] >> 5 | buf[22] << 3)                       & 0x07FF);

    for (i = 0U; i < SBUS_CHANNEL_NUM; i++) {
        data->channels_norm[i] = SBUS_NormalizeChannel(data->channels[i]);
    }

    data->frame_lost = (buf[23] & 0x04U) ? 1U : 0U;
    data->failsafe   = (buf[23] & 0x08U) ? 1U : 0U;
    data->updated    = 1U;
    data->rx_tick_ms = HAL_GetTick();
    {
        const uint32_t cycles_per_us = SystemCoreClock / 1000000U;
        data->rx_us = (cycles_per_us != 0U) ? (DWT->CYCCNT / cycles_per_us) : 0U;
    }
    sbus_dbg.frame_ok_cnt++;
    return 1;
}

void SBUS_FeedByte(uint8_t byte)
{
    if (sbus_rx_index < SBUS_FRAME_LEN) {
        sbus_rx_frame[sbus_rx_index++] = byte;
        if (sbus_rx_index < SBUS_FRAME_LEN) {
            return;
        }
    } else {
        memmove(sbus_rx_frame, &sbus_rx_frame[1], (size_t)(SBUS_FRAME_LEN - 1U));
        sbus_rx_frame[SBUS_FRAME_LEN - 1U] = byte;
    }

    if ((sbus_rx_frame[0] == SBUS_HEADER) &&
        (sbus_rx_frame[SBUS_FRAME_LEN - 1U] == SBUS_FOOTER)) {
        sbus_dbg.header_hit_cnt++;
        (void)SBUS_ParseFrame(sbus_rx_frame, &sbus_data);
        sbus_rx_index = 0U;
    }
}

void SBUS_OnDmaIdle(const volatile uint8_t *buf, uint16_t buf_len, uint16_t dma_pos)
{
    uint16_t i;

    if ((buf == NULL) || (buf_len == 0U)) {
        return;
    }
    if (dma_pos > buf_len) {
        dma_pos = buf_len;
    }

    if (dma_pos > sbus_dma_last_pos) {
        for (i = sbus_dma_last_pos; i < dma_pos; i++) {
            SBUS_FeedByte(buf[i]);
        }
    } else if (dma_pos < sbus_dma_last_pos) {
        for (i = sbus_dma_last_pos; i < buf_len; i++) {
            SBUS_FeedByte(buf[i]);
        }
        for (i = 0; i < dma_pos; i++) {
            SBUS_FeedByte(buf[i]);
        }
    }

    sbus_dma_last_pos = dma_pos;
    sbus_dbg.last_dma_pos = dma_pos;
}

void SBUS_PollFromUart(UART_HandleTypeDef *huart, const volatile uint8_t *buf, uint16_t buf_len)
{
    uint16_t dma_pos;

    if ((huart == NULL) || (huart->hdmarx == NULL) || (buf == NULL) || (buf_len == 0U)) {
        return;
    }

    sbus_dbg.rxinv_hw = (READ_BIT(huart->Instance->CR2, USART_CR2_RXINV) != 0U) ? 1U : 0U;

    dma_pos = buf_len - (uint16_t)__HAL_DMA_GET_COUNTER(huart->hdmarx);
    if (dma_pos > buf_len) {
        dma_pos = 0U;
    }
    sbus_dbg.dma_poll_cnt++;
    SBUS_OnDmaIdle(buf, buf_len, dma_pos);
}

void Debug_Printf(const char *fmt, ...)
{
    char buffer[256];
    va_list args;
    int len;

    va_start(args, fmt);
    len = vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);

    if (len <= 0) {
        return;
    }
    if (len > (int)sizeof(buffer)) {
        len = (int)sizeof(buffer);
    }
    HAL_UART_Transmit(&huart10, (uint8_t *)buffer, (uint16_t)len, HAL_MAX_DELAY);
}
