/**
  ******************************************************************************
  * @file    rtk_uart.h
  * @brief   UART7 接收 RTK/GNSS 模块 NMEA（GGA/RMC），供 INS 松耦合
  * @note    RTK 用 USB 独立供电，不接 UART7 座 VCC。
  *          模块 TX → PE7(RX)，模块 RX → PE8(TX，可选)，GND 必须共地。
  *          115200 8N1
  ******************************************************************************
  */

#ifndef RTK_UART_H
#define RTK_UART_H

#include <stdint.h>
#include "stm32h7xx_hal.h"

#define RTK_BAUDRATE        115200U
#define RTK_DMA_BUF_LEN     512U

typedef struct {
    double lat_deg;
    double lon_deg;
    double alt_m;
    float  vel_ned[3];     /* m/s，由 RMC 航向/航速推 NED */
    float  hdop;
    uint8_t quality;       /* GGA quality：4=RTK 固定，5=浮点 */
    uint8_t sats;
    uint8_t pos_valid;
    uint8_t vel_valid;
    uint8_t yaw_valid;     /* 双天线真北航向有效 */
    float  yaw_deg;        /* 双天线航向，度，真北顺时针 0～360 */
    uint32_t gga_cnt;
    uint32_t rmc_cnt;
    uint32_t hdt_cnt;      /* HDT/THS 解析次数 */
    uint32_t rx_bytes;     /* UART 收到的字节数，为 0 说明没接到数 */
    uint32_t line_cnt;     /* 收到的以 $ 开头的行 */
    uint32_t cs_fail;      /* 校验失败的行 */
    uint16_t dma_pos;      /* DMA 写入位置，有数时会走动 */
    uint8_t  last_byte;    /* 最近一个字节 */
    char     last_nmea[88]; /* 最近一行 NMEA，供 USART10 打印 */
} RTK_Fix_t;

extern UART_HandleTypeDef huart7;
extern DMA_HandleTypeDef hdma_uart7_rx;
extern RTK_Fix_t rtk_fix;
extern volatile uint8_t rtk_dma_buf[RTK_DMA_BUF_LEN];

void RTK_UART_Init(void);
void RTK_StartDmaRx(void);
void RTK_Poll(void);

#endif
