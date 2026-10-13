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
static char nmea_line[256];
static uint16_t nmea_len;
static uint8_t nmea_discard;
static uint16_t gsv_snr_sum;
static uint8_t  gsv_snr_n;
static uint8_t  gsv_snr_max;

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

static int is_leap(int y)
{
    return ((y % 4 == 0) && ((y % 100 != 0) || (y % 400 == 0))) ? 1 : 0;
}

static uint64_t civil_to_unix_us(int y, int mo, int d, int h, int mi, int s, uint32_t us)
{
    static const uint8_t dim[12] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
    int64_t days = 0;
    int yy;
    int mm;
    int64_t sec;

    if ((y < 1970) || (mo < 1) || (mo > 12) || (d < 1) || (d > 31) ||
        (h < 0) || (h > 23) || (mi < 0) || (mi > 59) || (s < 0) || (s > 60)) {
        return 0ULL;
    }
    for (yy = 1970; yy < y; yy++) {
        days += is_leap(yy) ? 366 : 365;
    }
    for (mm = 1; mm < mo; mm++) {
        days += dim[mm - 1];
        if ((mm == 2) && is_leap(y)) {
            days += 1;
        }
    }
    days += (d - 1);
    sec = days * 86400LL + (int64_t)h * 3600LL + (int64_t)mi * 60LL + (int64_t)s;
    if (sec < 0) {
        return 0ULL;
    }
    if (us >= 1000000U) {
        us = 999999U;
    }
    return (uint64_t)sec * 1000000ULL + (uint64_t)us;
}

static int parse_hhmmss(const char *t, uint8_t *h, uint8_t *mi, uint8_t *s, uint32_t *us)
{
    unsigned hh;
    unsigned mm;
    unsigned ss;
    unsigned whole;
    double v;
    double frac;
    int i;

    if ((t == NULL) || (t[0] == '\0')) {
        return 0;
    }
    for (i = 0; i < 6; i++) {
        if ((t[i] < '0') || (t[i] > '9')) {
            return 0;
        }
    }
    v = atof(t);
    whole = (unsigned)v;
    hh = whole / 10000U;
    mm = (whole / 100U) % 100U;
    ss = whole % 100U;
    if ((hh > 23U) || (mm > 59U) || (ss > 60U)) {
        return 0;
    }
    *h = (uint8_t)hh;
    *mi = (uint8_t)mm;
    *s = (uint8_t)ss;
    frac = v - (double)whole;
    if (frac < 0.0) {
        frac = 0.0;
    }
    *us = (uint32_t)(frac * 1000000.0 + 0.5);
    if (*us >= 1000000U) {
        *us = 999999U;
    }
    return 1;
}

static int parse_ddmmyy(const char *t, uint16_t *y, uint8_t *mo, uint8_t *d)
{
    unsigned dd;
    unsigned mm;
    unsigned yy;
    int i;

    if ((t == NULL) || (t[0] == '\0')) {
        return 0;
    }
    for (i = 0; i < 6; i++) {
        if ((t[i] < '0') || (t[i] > '9')) {
            return 0;
        }
    }
    dd = (unsigned)((t[0] - '0') * 10 + (t[1] - '0'));
    mm = (unsigned)((t[2] - '0') * 10 + (t[3] - '0'));
    yy = (unsigned)((t[4] - '0') * 10 + (t[5] - '0'));
    if ((dd < 1U) || (dd > 31U) || (mm < 1U) || (mm > 12U)) {
        return 0;
    }
    *d = (uint8_t)dd;
    *mo = (uint8_t)mm;
    *y = (uint16_t)(2000U + yy);
    return 1;
}

static void rtk_unix_refresh(void)
{
    uint64_t us;

    if ((rtk_fix.date_valid == 0U) || (rtk_fix.time_valid == 0U)) {
        return;
    }
    us = civil_to_unix_us((int)rtk_fix.utc_year, (int)rtk_fix.utc_mon, (int)rtk_fix.utc_day,
                          (int)rtk_fix.utc_hour, (int)rtk_fix.utc_min, (int)rtk_fix.utc_sec,
                          rtk_fix.utc_usec);
    if (us == 0ULL) {
        rtk_fix.unix_valid = 0U;
        return;
    }
    rtk_fix.unix_us = us;
    rtk_fix.unix_tick_ms = HAL_GetTick();
    rtk_fix.unix_valid = 1U;
}

static void parse_nmea_time_field(const char *s, int idx)
{
    char t[16];

    nmea_field(s, idx, t, sizeof(t));
    if (parse_hhmmss(t, &rtk_fix.utc_hour, &rtk_fix.utc_min, &rtk_fix.utc_sec, &rtk_fix.utc_usec)) {
        rtk_fix.time_valid = 1U;
        rtk_unix_refresh();
    }
}

uint64_t RTK_UnixNowUs(void)
{
    uint32_t dt;

    if (rtk_fix.unix_valid == 0U) {
        return 0ULL;
    }
    dt = HAL_GetTick() - rtk_fix.unix_tick_ms;
    return rtk_fix.unix_us + ((uint64_t)dt * 1000ULL);
}

static void parse_gga(const char *s)
{
    char lat[24], ns[4], lon[24], ew[4], q[8], alt[16], hdop[16], sat[8];
    uint8_t quality;

    parse_nmea_time_field(s, 1);
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
    /* 上一历元 GSV 的载噪比在本帧 GGA 处结算，避免多星座互相覆盖 */
    if (gsv_snr_n > 0U) {
        rtk_fix.snr_avg = (float)gsv_snr_sum / (float)gsv_snr_n;
        rtk_fix.snr_max = gsv_snr_max;
        gsv_snr_sum = 0U;
        gsv_snr_n = 0U;
        gsv_snr_max = 0U;
    }
    rtk_fix.gga_cnt++;
}

static void parse_gsv(const char *s)
{
    char snr[8];
    int i;
    int snr_v;

    /* 每句最多 4 颗星：字段 7/11/15/19 为 SNR(dB-Hz) */
    for (i = 0; i < 4; i++) {
        nmea_field(s, 7 + i * 4, snr, sizeof(snr));
        if (snr[0] == '\0') {
            continue;
        }
        snr_v = atoi(snr);
        if (snr_v <= 0) {
            continue;
        }
        gsv_snr_sum = (uint16_t)(gsv_snr_sum + (uint16_t)snr_v);
        gsv_snr_n++;
        if (snr_v > (int)gsv_snr_max) {
            gsv_snr_max = (uint8_t)snr_v;
        }
    }
}

static void parse_rmc(const char *s)
{
    char st[4], spd[16], cog[16], date[12];
    float knots, course_deg, vn, ve;

    parse_nmea_time_field(s, 1);
    nmea_field(s, 9, date, sizeof(date));
    if (parse_ddmmyy(date, &rtk_fix.utc_year, &rtk_fix.utc_mon, &rtk_fix.utc_day)) {
        rtk_fix.date_valid = 1U;
        rtk_unix_refresh();
    }

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
    } else if (strstr(line, "GSV") != NULL) {
        parse_gsv(line);
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
        if ((nmea_discard == 0U) && (nmea_len >= 6U)) {
            nmea_line[nmea_len] = '\0';
            nmea_handle_line(nmea_line);
        }
        nmea_len = 0U;
        nmea_discard = 0U;
        return;
    }
    if (nmea_discard != 0U) {
        return;
    }
    if (nmea_len < (sizeof(nmea_line) - 1U)) {
        if ((nmea_len == 0U) && (b != '$')) {
            return;
        }
        nmea_line[nmea_len++] = (char)b;
    } else {
        /* 超长句丢到行尾，避免截断后把后半段当成新句子 */
        nmea_discard = 1U;
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
