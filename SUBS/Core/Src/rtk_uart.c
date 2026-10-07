/**
  ******************************************************************************
  * @file    rtk_uart.c
  * @brief   UART7 DMA 循环收 NMEA，解析 GGA 位置与 RMC 速度
  *          PE7=RX、PE8=TX；模块 USB 供电，不接板载 VCC
  ******************************************************************************
  */

#include "rtk_uart.h"
#include "main.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

UART_HandleTypeDef huart7;
DMA_HandleTypeDef hdma_uart7_rx;
RTK_Fix_t rtk_fix;

#if defined(__CC_ARM)
volatile uint8_t rtk_dma_buf[RTK_DMA_BUF_LEN] __attribute__((section("RAM_D1"), zero_init, aligned(32)));
#else
volatile uint8_t rtk_dma_buf[RTK_DMA_BUF_LEN] __attribute__((section("RAM_D1"), aligned(32)));
#endif

static uint16_t rtk_dma_last_pos;
static char nmea_line[128];
static uint8_t nmea_len;

static int hex_nibble(char c)
{
    if ((c >= '0') && (c <= '9')) {
        return c - '0';
    }
    if ((c >= 'A') && (c <= 'F')) {
        return c - 'A' + 10;
    }
    if ((c >= 'a') && (c <= 'f')) {
        return c - 'a' + 10;
    }
    return -1;
}

static int nmea_checksum_ok(const char *s)
{
    const char *p;
    uint8_t cs = 0;
    int hi, lo;

    if (s[0] != '$') {
        return 0;
    }
    for (p = s + 1; *p != '\0' && *p != '*'; p++) {
        cs ^= (uint8_t)(*p);
    }
    /* 无校验字段时仍尝试解析，避免模块未开 checksum 时全丢 */
    if (*p != '*') {
        return 1;
    }
    hi = hex_nibble(p[1]);
    lo = hex_nibble(p[2]);
    if ((hi < 0) || (lo < 0)) {
        return 0;
    }
    return (cs == (uint8_t)((hi << 4) | lo)) ? 1 : 0;
}

/* ddmm.mmmm → 度 */
static double nmea_degmin_to_deg(const char *f, char hemi)
{
    double v, deg, minv;
    if ((f == NULL) || (f[0] == '\0')) {
        return 0.0;
    }
    v = atof(f);
    deg = floor(v / 100.0);
    minv = v - deg * 100.0;
    v = deg + minv / 60.0;
    if ((hemi == 'S') || (hemi == 'W')) {
        v = -v;
    }
    return v;
}

static int nmea_field(const char *s, int idx, char *out, int outsz)
{
    const char *p = s;
    int n = 0;
    int i;

    while (*p && n < idx) {
        if (*p++ == ',') {
            n++;
        }
    }
    i = 0;
    while (*p && *p != ',' && *p != '*' && i < (outsz - 1)) {
        out[i++] = *p++;
    }
    out[i] = '\0';
    return i;
}

static void parse_gga(const char *s)
{
    char lat[24], ns[4], lon[24], ew[4], q[8], alt[16], hdop[16], sat[8];
    uint8_t quality;

    nmea_field(s, 2, lat, sizeof(lat));
    nmea_field(s, 3, ns, sizeof(ns));
    nmea_field(s, 4, lon, sizeof(lon));
    nmea_field(s, 5, ew, sizeof(ew));
    nmea_field(s, 6, q, sizeof(q));
    nmea_field(s, 7, sat, sizeof(sat));
    nmea_field(s, 8, hdop, sizeof(hdop));
    nmea_field(s, 9, alt, sizeof(alt));

    quality = (uint8_t)atoi(q);
    rtk_fix.lat_deg = nmea_degmin_to_deg(lat, ns[0]);
    rtk_fix.lon_deg = nmea_degmin_to_deg(lon, ew[0]);
    rtk_fix.quality = quality;
    rtk_fix.sats = (uint8_t)atoi(sat);
    rtk_fix.hdop = (float)atof(hdop);
    rtk_fix.alt_m = atof(alt);
    rtk_fix.pos_valid = (quality >= 1U) ? 1U : 0U;
    rtk_fix.gga_cnt++;
}

static void parse_rmc(const char *s)
{
    char st[4], spd[16], cog[16];
    float knots, course_deg, vn, ve;

    nmea_field(s, 2, st, sizeof(st));
    if (st[0] != 'A') {
        rtk_fix.vel_valid = 0U;
        return;
    }
    nmea_field(s, 7, spd, sizeof(spd));
    nmea_field(s, 8, cog, sizeof(cog));
    knots = (float)atof(spd);
    course_deg = (float)atof(cog);
    vn = (knots * 0.514444f) * (float)cos((double)course_deg * M_PI / 180.0);
    ve = (knots * 0.514444f) * (float)sin((double)course_deg * M_PI / 180.0);
    rtk_fix.vel_ned[0] = vn;
    rtk_fix.vel_ned[1] = ve;
    rtk_fix.vel_ned[2] = 0.0f;
    rtk_fix.vel_valid = 1U;
    rtk_fix.rmc_cnt++;
}

static void parse_hdt(const char *s)
{
    char hdg[24];
    float deg;

    nmea_field(s, 1, hdg, sizeof(hdg));
    if (hdg[0] == '\0') {
        rtk_fix.yaw_valid = 0U;
        return;
    }
    deg = (float)atof(hdg);
    if ((deg < 0.0f) || (deg > 360.0f)) {
        rtk_fix.yaw_valid = 0U;
        return;
    }
    rtk_fix.yaw_deg = deg;
    rtk_fix.yaw_valid = 1U;
    rtk_fix.hdt_cnt++;
}

static void parse_ths(const char *s)
{
    char st[4];

    nmea_field(s, 2, st, sizeof(st));
    /* A=自主有效，E=估计；V=无效 */
    if ((st[0] != 'A') && (st[0] != 'E') && (st[0] != '\0')) {
        if (st[0] == 'V') {
            rtk_fix.yaw_valid = 0U;
        }
        return;
    }
    parse_hdt(s); /* 第 1 字段同样是航向角 */
}

static void nmea_handle_line(char *line)
{
    uint32_t i;

    rtk_fix.line_cnt++;
    for (i = 0U; (i < (sizeof(rtk_fix.last_nmea) - 1U)) && (line[i] != '\0'); i++) {
        rtk_fix.last_nmea[i] = line[i];
    }
    rtk_fix.last_nmea[i] = '\0';
    if (!nmea_checksum_ok(line)) {
        rtk_fix.cs_fail++;
        return;
    }
    if (strstr(line, "GGA") != NULL) {
        parse_gga(line);
    } else if (strstr(line, "RMC") != NULL) {
        parse_rmc(line);
    } else if (strstr(line, "HDT") != NULL) {
        parse_hdt(line);
    } else if (strstr(line, "THS") != NULL) {
        parse_ths(line);
    }
}

static void nmea_feed(uint8_t b)
{
    rtk_fix.rx_bytes++;
    rtk_fix.last_byte = b;
    if ((b == '\r') || (b == '\n')) {
        if (nmea_len >= 6U) {
            nmea_line[nmea_len] = '\0';
            nmea_handle_line(nmea_line);
        }
        nmea_len = 0U;
        return;
    }
    if (nmea_len < (sizeof(nmea_line) - 1U)) {
        if ((nmea_len == 0U) && (b != '$')) {
            return;
        }
        nmea_line[nmea_len++] = (char)b;
    } else {
        nmea_len = 0U;
    }
}

void RTK_UART_Init(void)
{
    huart7.Instance = UART7;
    huart7.Init.BaudRate = RTK_BAUDRATE;
    huart7.Init.WordLength = UART_WORDLENGTH_8B;
    huart7.Init.StopBits = UART_STOPBITS_1;
    huart7.Init.Parity = UART_PARITY_NONE;
    huart7.Init.Mode = UART_MODE_TX_RX;
    huart7.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart7.Init.OverSampling = UART_OVERSAMPLING_16;
    huart7.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    huart7.Init.ClockPrescaler = UART_PRESCALER_DIV1;
    huart7.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    if (HAL_UART_Init(&huart7) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_UARTEx_SetTxFifoThreshold(&huart7, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_UARTEx_SetRxFifoThreshold(&huart7, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_UARTEx_DisableFifoMode(&huart7) != HAL_OK) {
        Error_Handler();
    }
}

void RTK_StartDmaRx(void)
{
    __HAL_UART_CLEAR_FLAG(&huart7, UART_CLEAR_IDLEF | UART_CLEAR_PEF |
                          UART_CLEAR_FEF | UART_CLEAR_NEF | UART_CLEAR_OREF);
    if (HAL_UARTEx_ReceiveToIdle_DMA(&huart7, (uint8_t *)rtk_dma_buf, RTK_DMA_BUF_LEN) != HAL_OK) {
        return;
    }
    __HAL_DMA_DISABLE_IT(huart7.hdmarx, DMA_IT_HT);
    ATOMIC_CLEAR_BIT(huart7.Instance->CR1, USART_CR1_PEIE);
    ATOMIC_CLEAR_BIT(huart7.Instance->CR3, USART_CR3_EIE);
    huart7.ReceptionType = HAL_UART_RECEPTION_TOIDLE;
    __HAL_UART_CLEAR_FLAG(&huart7, UART_CLEAR_IDLEF);
    ATOMIC_SET_BIT(huart7.Instance->CR1, USART_CR1_IDLEIE);
}

void RTK_Poll(void)
{
    uint16_t pos;
    uint16_t last;

    if (huart7.hdmarx == NULL) {
        return;
    }
    pos = RTK_DMA_BUF_LEN - (uint16_t)__HAL_DMA_GET_COUNTER(huart7.hdmarx);
    rtk_fix.dma_pos = pos;
    last = rtk_dma_last_pos;
    while (last != pos) {
        nmea_feed(rtk_dma_buf[last]);
        last++;
        if (last >= RTK_DMA_BUF_LEN) {
            last = 0U;
        }
    }
    rtk_dma_last_pos = pos;
}
