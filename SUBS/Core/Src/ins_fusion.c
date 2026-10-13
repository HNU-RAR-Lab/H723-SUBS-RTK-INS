/**
  ******************************************************************************
  * @file    ins_fusion.c
  * @brief   BMI088 + RTK NMEA 松耦合，对接 INSLIB 的 ins_init / ins_update / ins_get_*
  *
  * 数据流：
  *   BMI088（比力、角速度）约 100 Hz 做捷联预测；
  *   UART7 NMEA 的 GGA 位置、RMC 速度、HDT 双天线航向在新帧到达时作为量测更新。
  * 同一条 NMEA 用 gga_cnt / rmc_cnt / hdt_cnt 去重，避免重复融合。
  *
  * 滤波本身在 INSLIB（15 状态 ESKF，UDU 协方差）。本文件只组 ins_measurements_t。
 * USART10 的航向打印取 HDT/THS 或 #UNIHEADINGA 真北航向，不取融合 yaw。
  ******************************************************************************
  */

#include "ins_fusion.h"
#include "bmi088.h"
#include "rtk_uart.h"
#include "sbus.h"              /* Debug_Printf，调试口 USART10 */
#include "main.h"              /* HAL_GetTick */
#include "ins.h"
#include "geodetic_toolbox.h"  /* DEG2RAD / RAD2DEG */
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <math.h>

/** 对外融合结果，定义见 ins_fusion.h */
INS_FusionOut_t ins_out;

/*
 * INSLIB 状态体较大且每步都访问，放在 STM32H7 的 D1 域 SRAM（RAM_D1）。
 * Keil ARMCC 需要 zero_init，否则带 section 的静态变量不一定进 BSS 清零。
 */
#if defined(__CC_ARM)
static ins_t g_ins __attribute__((section("RAM_D1"), zero_init));
#else
static ins_t g_ins __attribute__((section("RAM_D1")));
#endif

static uint8_t  g_inited;       /**< 1：ins_init 成功，Poll 才允许跑滤波 */
static uint32_t g_last_imu_ms;  /**< 上次 IMU/滤波步进的 HAL_GetTick，单位 ms */
static uint32_t g_last_gga;     /**< 已融合过的 GGA 计数，与 rtk_fix.gga_cnt 比较 */
static uint32_t g_last_rmc;     /**< 已融合过的 RMC 计数 */
static uint32_t g_last_hdt;     /**< 已融合过的 HDT/THS 计数 */

/**
 * @brief  当前时刻，微秒
 * @note   HAL_GetTick 分辨率为 1 ms，这里只是换成 INSLIB 要求的 us 时间基。
 */
static ins_time_us_t now_us(void)
{
    return (ins_time_us_t)HAL_GetTick() * 1000LL;
}

/**
 * @brief  由 GGA 定位质量与 HDOP 估计 GNSS 位置 1σ（米）
 * @param  q    GGA quality
 * @param  hdop 水平精度因子；小于 0.3 时按 1.0 用，避免过小权重
 * @param  sh   输出：北、东向标准差，米
 * @param  sv   输出：地向标准差，米
 *
 * quality 4 固定解约 3 cm / 6 cm；5 浮点解约 30 cm / 60 cm。
 * 2 与其它模式按 HDOP 放大，垂向约为水平的 2 倍。
 * 调用方把返回值平方后写入 Qll_ned 对角。
 */
static void rtk_sigma(uint8_t q, float hdop, float *sh, float *sv)
{
    float h = (hdop > 0.3f) ? hdop : 1.0f;

    switch (q) {
    case 4:  *sh = 0.03f; *sv = 0.06f; break;       /* RTK 固定 */
    case 5:  *sh = 0.30f; *sv = 0.60f; break;       /* RTK 浮点 */
    case 2:  *sh = 1.0f * h; *sv = 2.0f * h; break; /* 差分等 */
    default: *sh = 2.5f * h; *sv = 5.0f * h; break; /* 单点或未知 */
    }
}

static float wrap_deg180(float deg)
{
    while (deg > 180.0f) {
        deg -= 360.0f;
    }
    while (deg < -180.0f) {
        deg += 360.0f;
    }
    return deg;
}

/* P = U*diag(d)*U'，取对角方差。U 按列优先，与 INSLIB MAT_ELEM 一致 */
static float udu_var(int idx)
{
    int k;
    int n;
    float s;
    float u;

    n = g_ins.n;
    if ((idx < 0) || (idx >= n)) {
        return 0.0f;
    }
    s = g_ins.d[idx];
    for (k = idx + 1; k < n; k++) {
        u = g_ins.U[idx + k * n];
        s += u * u * g_ins.d[k];
    }
    if (s < 0.0f) {
        s = 0.0f;
    }
    return s;
}

static void fmt_u64(char *out, uint32_t outsz, uint64_t v)
{
    char tmp[24];
    int n = 0;
    int i;

    if ((out == NULL) || (outsz < 2U)) {
        return;
    }
    if (v == 0ULL) {
        out[0] = '0';
        out[1] = '\0';
        return;
    }
    while ((v > 0ULL) && (n < 20)) {
        tmp[n++] = (char)('0' + (unsigned)(v % 10ULL));
        v /= 10ULL;
    }
    if (n >= (int)outsz) {
        n = (int)outsz - 1;
    }
    for (i = 0; i < n; i++) {
        out[i] = tmp[n - 1 - i];
    }
    out[n] = '\0';
}

static void ins_fusion_refresh_nav(double llh[3], float *roll, float *pitch, float *yaw)
{
    float pn, pe, pd;
    float vn, ve, vd;
    float yaw_sd;
    float roll_sd;
    float pitch_sd;
    int dr_ms;

    if (ins_out.ready == 0U) {
        ins_out.xy_valid = 0U;
        ins_out.z_valid = 0U;
        ins_out.v_xy_valid = 0U;
        ins_out.v_z_valid = 0U;
        ins_out.heading_good_for_control = 0U;
        ins_out.dead_reckoning = 0U;
        return;
    }

    if (ins_get_latlonh(&g_ins, llh)) {
        ins_out.lat_deg = RAD2DEG(llh[0]);
        ins_out.lon_deg = RAD2DEG(llh[1]);
        ins_out.h_m = llh[2];
    }
    (void)ins_get_velocity_ned(&g_ins, ins_out.vel_ned);
    if (ins_get_rpy(&g_ins, roll, pitch, yaw)) {
        ins_out.roll_deg = (float)RAD2DEG(*roll);
        ins_out.pitch_deg = (float)RAD2DEG(*pitch);
        ins_out.yaw_deg = (float)RAD2DEG(*yaw);
        ins_out.heading_deg = wrap_deg180(ins_out.yaw_deg);
    }
    (void)ins_get_position_local(&g_ins, ins_out.pos_ned);
    (void)ins_get_acc_n(&g_ins, ins_out.acc_ned);

    ins_out.ref_lat_deg = RAD2DEG(g_ins.origin_llh[0]);
    ins_out.ref_lon_deg = RAD2DEG(g_ins.origin_llh[1]);
    ins_out.ref_alt_m = g_ins.origin_llh[2];

    pn = udu_var(INS_IDX_POS + 0);
    pe = udu_var(INS_IDX_POS + 1);
    pd = udu_var(INS_IDX_POS + 2);
    vn = udu_var(INS_IDX_VEL + 0);
    ve = udu_var(INS_IDX_VEL + 1);
    vd = udu_var(INS_IDX_VEL + 2);
    ins_out.eph = sqrtf(pn + pe);
    ins_out.epv = sqrtf(pd);
    ins_out.evh = sqrtf(vn + ve);
    ins_out.evv = sqrtf(vd);

    if (ins_get_rpy_stddev(&g_ins, &roll_sd, &pitch_sd, &yaw_sd)) {
        (void)roll_sd;
        (void)pitch_sd;
        ins_out.heading_var = yaw_sd * yaw_sd;
        ins_out.heading_good_for_control =
            ((yaw_sd < 0.087f) && (rtk_fix.yaw_valid != 0U)) ? 1U : 0U;
    } else {
        ins_out.heading_var = 0.0f;
        ins_out.heading_good_for_control = 0U;
    }

    ins_out.xy_valid = 1U;
    ins_out.z_valid = 1U;
    ins_out.v_xy_valid = 1U;
    ins_out.v_z_valid = 1U;

    dr_ms = ins_deadreckoning_ms(&g_ins);
    ins_out.dead_reckoning = ((dr_ms > 300) && (rtk_fix.pos_valid == 0U)) ? 1U : 0U;
}

/* ARMCC5 不能在字面量里写 UTF-8 汉字，用十六进制字节 */
#define CN_TS       "\xE6\x97\xB6\xE9\x97\xB4\xE6\x88\xB3"
#define CN_TS_SAMP  "\xE9\x87\x87\xE6\xA0\xB7\xE6\x97\xB6\xE9\x97\xB4\xE6\x88\xB3"
#define CN_XY_VALID "\xE6\xB0\xB4\xE5\xB9\xB3\xE4\xBD\x8D\xE7\xBD\xAE\xE6\x9C\x89\xE6\x95\x88"
#define CN_Z_VALID  "\xE9\xAB\x98\xE5\xBA\xA6\xE6\x9C\x89\xE6\x95\x88"
#define CN_VXY_VALID "\xE6\xB0\xB4\xE5\xB9\xB3\xE9\x80\x9F\xE5\xBA\xA6\xE6\x9C\x89\xE6\x95\x88"
#define CN_VZ_VALID "\xE5\x9E\x82\xE7\x9B\xB4\xE9\x80\x9F\xE5\xBA\xA6\xE6\x9C\x89\xE6\x95\x88"
#define CN_X        "\xE5\x8C\x97\xE5\x90\x91\xE4\xBD\x8D\xE7\xBD\xAE"
#define CN_Y        "\xE4\xB8\x9C\xE5\x90\x91\xE4\xBD\x8D\xE7\xBD\xAE"
#define CN_Z        "\xE5\x9C\xB0\xE5\x90\x91\xE4\xBD\x8D\xE7\xBD\xAE"
#define CN_VX       "\xE5\x8C\x97\xE5\x90\x91\xE9\x80\x9F\xE5\xBA\xA6"
#define CN_VY       "\xE4\xB8\x9C\xE5\x90\x91\xE9\x80\x9F\xE5\xBA\xA6"
#define CN_VZ       "\xE5\x9C\xB0\xE5\x90\x91\xE9\x80\x9F\xE5\xBA\xA6"
#define CN_AX       "\xE5\x8C\x97\xE5\x90\x91\xE5\x8A\xA0\xE9\x80\x9F\xE5\xBA\xA6"
#define CN_AY       "\xE4\xB8\x9C\xE5\x90\x91\xE5\x8A\xA0\xE9\x80\x9F\xE5\xBA\xA6"
#define CN_AZ       "\xE5\x9C\xB0\xE5\x90\x91\xE5\x8A\xA0\xE9\x80\x9F\xE5\xBA\xA6"
#define CN_RTK_HDG  "RTK\xE8\x88\xAA\xE5\x90\x91"
#define CN_HDG_OK   "\xE8\x88\xAA\xE5\x90\x91\xE6\x9C\x89\xE6\x95\x88"

/* 超过该间隔没有新的 HDT/THS，打印标为无效，避免停更后仍像活航向 */
#define RTK_HDG_STALE_MS  2000U
#define CN_REF_LAT  "\xE5\x8E\x9F\xE7\x82\xB9\xE7\xBA\xAC\xE5\xBA\xA6"
#define CN_REF_LON  "\xE5\x8E\x9F\xE7\x82\xB9\xE7\xBB\x8F\xE5\xBA\xA6"
#define CN_REF_ALT  "\xE5\x8E\x9F\xE7\x82\xB9\xE9\xAB\x98\xE5\xBA\xA6"
#define CN_EPH      "\xE6\xB0\xB4\xE5\xB9\xB3\xE4\xBD\x8D\xE7\xBD\xAE\xE7\xB2\xBE\xE5\xBA\xA6"
#define CN_EPV      "\xE5\x9E\x82\xE7\x9B\xB4\xE4\xBD\x8D\xE7\xBD\xAE\xE7\xB2\xBE\xE5\xBA\xA6"
#define CN_EVH      "\xE6\xB0\xB4\xE5\xB9\xB3\xE9\x80\x9F\xE5\xBA\xA6\xE7\xB2\xBE\xE5\xBA\xA6"
#define CN_EVV      "\xE5\x9E\x82\xE7\x9B\xB4\xE9\x80\x9F\xE5\xBA\xA6\xE7\xB2\xBE\xE5\xBA\xA6"
#define CN_DR       "\xE6\x8E\xA8\xE7\xAE\x97\xE8\x88\xAA\xE8\xA1\x8C"
#define CN_YES      "\xE6\x98\xAF"
#define CN_NO       "\xE5\x90\xA6"
#define CN_ROLL     "\xE6\xBB\x9A\xE8\xBD\xAC"
#define CN_PITCH    "\xE4\xBF\xAF\xE4\xBB\xB0"
#define CN_READY    "\xE8\x9E\x8D\xE5\x90\x88\xE5\xB0\xB1\xE7\xBB\xAA"
#define CN_RAW_LAT  "\xE5\x8E\x9F\xE5\xA7\x8B\xE7\xBA\xAC\xE5\xBA\xA6"
#define CN_RAW_LON  "\xE5\x8E\x9F\xE5\xA7\x8B\xE7\xBB\x8F\xE5\xBA\xA6"
#define CN_RAW_ALT  "\xE5\x8E\x9F\xE5\xA7\x8B\xE6\xB5\xB7\xE6\x8B\x94"
#define CN_QUAL     "\xE5\xAE\x9A\xE4\xBD\x8D\xE8\xB4\xA8\xE9\x87\x8F"
#define CN_RXB      "\xE6\x8E\xA5\xE6\x94\xB6\xE5\xAD\x97\xE8\x8A\x82"
#define CN_SATS     "\xE5\x8F\x82\xE4\xB8\x8E\xE8\xA7\xA3\xE7\xAE\x97\xE5\x8D\xAB\xE6\x98\x9F\xE6\x95\xB0"
#define CN_HDOP     "\xE6\xB0\xB4\xE5\xB9\xB3\xE7\xB2\xBE\xE5\xBA\xA6\xE5\x9B\xA0\xE5\xAD\x90"
#define CN_SNR      "\xE6\x8E\xA5\xE6\x94\xB6\xE5\xBC\xBA\xE5\xBA\xA6"
#define CN_SNR_MAX  "\xE6\x9C\x80\xE5\xBC\xBA\xE6\x8E\xA5\xE6\x94\xB6"

static const char *cn_yn(uint8_t v)
{
    return (v != 0U) ? CN_YES : CN_NO;
}

/* 双天线真北航向：只认最近一帧 HDT/THS，与融合 yaw、融合是否就绪无关 */
static uint8_t rtk_heading_live(void)
{
    uint32_t age;

    if (rtk_fix.yaw_valid == 0U) {
        return 0U;
    }
    age = HAL_GetTick() - rtk_fix.yaw_tick_ms;
    return (age <= RTK_HDG_STALE_MS) ? 1U : 0U;
}

/* 经纬度非 0 时在数值后加 N/S、E/W；为 0 时不加方位 */
static void fmt_latlon(char *out, uint32_t outsz, double deg, char pos_h, char neg_h)
{
    if (deg == 0.0) {
        (void)snprintf(out, outsz, "%.8f", 0.0);
    } else if (deg > 0.0) {
        (void)snprintf(out, outsz, "%.8f, %c", deg, pos_h);
    } else {
        (void)snprintf(out, outsz, "%.8f, %c", -deg, neg_h);
    }
}

static void fusion_print_line(const char *fmt, ...)
{
    char line[320];
    va_list ap;
    int n;

    va_start(ap, fmt);
    n = vsnprintf(line, sizeof(line), fmt, ap);
    va_end(ap);
    if (n < 0) {
        return;
    }
    if (n >= (int)sizeof(line)) {
        n = (int)sizeof(line) - 1;
    }
    Debug_Write((uint8_t *)line, (uint16_t)n);
}

void INS_Fusion_DebugPrint(void)
{
    char ts[24];
    char ts_s[24];
    char lat_s[28];
    char lon_s[28];
    char raw_lat[28];
    char raw_lon[28];

    ins_out.timestamp_us = RTK_UnixNowUs();
    fmt_u64(ts, sizeof(ts), ins_out.timestamp_us);
    fmt_u64(ts_s, sizeof(ts_s), ins_out.timestamp_sample_us);
    fmt_latlon(lat_s, sizeof(lat_s), ins_out.ref_lat_deg, 'N', 'S');
    fmt_latlon(lon_s, sizeof(lon_s), ins_out.ref_lon_deg, 'E', 'W');
    fmt_latlon(raw_lat, sizeof(raw_lat), rtk_fix.lat_deg, 'N', 'S');
    fmt_latlon(raw_lon, sizeof(raw_lon), rtk_fix.lon_deg, 'E', 'W');

    /* 拆成多行，避免单帧 >512 字节时 ARMCC5 snprintf 返回 -1 整包丢弃 */
    fusion_print_line(CN_TS ": %s " CN_TS_SAMP ": %s " CN_READY ": %s\r\n",
                      ts, ts_s, cn_yn(ins_out.ready));
    /* 仅打印 RTK 已有量：GGA 卫星数/HDOP，GSV 载噪比。无干扰、增益字段 */
    fusion_print_line(CN_SATS ": %u " CN_HDOP ": %.1f " CN_SNR ": %.1f " CN_SNR_MAX ": %u\r\n",
                      (unsigned)rtk_fix.sats, rtk_fix.hdop,
                      rtk_fix.snr_avg, (unsigned)rtk_fix.snr_max);
    /* 原始 GGA，不看融合是否就绪。接收字节不增加说明 UART7 没数 */
    fusion_print_line(CN_RAW_LAT ": %s " CN_RAW_LON ": %s " CN_RAW_ALT ": %.3f "
                      CN_QUAL ": %u " CN_RXB ": %lu GGA:%lu\r\n",
                      raw_lat, raw_lon, rtk_fix.alt_m,
                      (unsigned)rtk_fix.quality,
                      (unsigned long)rtk_fix.rx_bytes,
                      (unsigned long)rtk_fix.gga_cnt);
    fusion_print_line(CN_XY_VALID ": %s " CN_Z_VALID ": %s " CN_VXY_VALID ": %s " CN_VZ_VALID ": %s\r\n",
                      cn_yn(ins_out.xy_valid), cn_yn(ins_out.z_valid),
                      cn_yn(ins_out.v_xy_valid), cn_yn(ins_out.v_z_valid));
    fusion_print_line(CN_X ": %.3f " CN_Y ": %.3f " CN_Z ": %.3f "
                      CN_VX ": %.3f " CN_VY ": %.3f " CN_VZ ": %.3f\r\n",
                      ins_out.pos_ned[0], ins_out.pos_ned[1], ins_out.pos_ned[2],
                      ins_out.vel_ned[0], ins_out.vel_ned[1], ins_out.vel_ned[2]);
    fusion_print_line(CN_AX ": %.3f " CN_AY ": %.3f " CN_AZ ": %.3f "
                      CN_ROLL ": %.1f " CN_PITCH ": %.1f\r\n",
                      ins_out.acc_ned[0], ins_out.acc_ned[1], ins_out.acc_ned[2],
                      ins_out.roll_deg, ins_out.pitch_deg);
    /* 真北 0～360°，来自 HDT/THS 或 #UNIHEADINGA；融合未就绪时同样打印 */
    fusion_print_line(CN_RTK_HDG ": %.2f " CN_HDG_OK ": %s HDG:%lu\r\n",
                      rtk_fix.yaw_deg, cn_yn(rtk_heading_live()),
                      (unsigned long)rtk_fix.hdt_cnt);
    fusion_print_line(CN_REF_LAT ": %s " CN_REF_LON ": %s " CN_REF_ALT ": %.3f "
                      CN_EPH ": %.3f " CN_EPV ": %.3f " CN_EVH ": %.3f " CN_EVV ": %.3f "
                      CN_DR ": %s\r\n\r\n",
                      lat_s, lon_s, ins_out.ref_alt_m,
                      ins_out.eph, ins_out.epv, ins_out.evh, ins_out.evv,
                      cn_yn(ins_out.dead_reckoning));
}

/**
 * @brief 上电初始化：BMI088、UART7 DMA 收 NMEA、INSLIB
 *
 * 初始经纬高只是占位（约 31°N、121°E、10 m）。opt.auto_init 为真时，
 * 库会用第一组可用的 IMU+GNSS 重新对齐原点和姿态。
 * GNSS 启动门限故意放宽，单点定位也能进滤波；RTK 固定后由 rtk_sigma
 * 把位置量测噪声收紧。
 * ins_init 失败时 g_inited 保持 0，INS_Fusion_Poll 不再调用 ins_update。
 */
void INS_Fusion_Init(void)
{
    ins_init_t init;
    ins_options_t opt;
    BMI088_Sample_t s;

    memset(&g_ins, 0, sizeof(g_ins));
    memset(&init, 0, sizeof(init));
    memset(&opt, 0, sizeof(opt));
    memset(&ins_out, 0, sizeof(ins_out));

    /* 芯片 ID 打印到 USART10，便于确认 SPI 是否读到 BMI088 */
    if (BMI088_Init() == 0) {
        ins_out.imu_ok = 1U;
        (void)BMI088_Read(&s);
        Debug_Printf("BMI088 OK id ACC=0x%02X GYRO=0x%02X\r\n",
                     (unsigned)bmi088.acc_id, (unsigned)bmi088.gyro_id);
    } else {
        ins_out.imu_ok = 0U;
        Debug_Printf("BMI088 fail id ACC=0x%02X GYRO=0x%02X\r\n",
                     (unsigned)bmi088.acc_id, (unsigned)bmi088.gyro_id);
    }

    /* RTK 模块 TX → PE7，115200 8N1，DMA 循环接收 */
    RTK_UART_Init();
    RTK_StartDmaRx();

    /* 占位原点与初始不确定度；auto_init 成功后会被第一组有效观测替换 */
    init.llh[0] = DEG2RAD(31.0);                    /* 纬度，rad */
    init.llh[1] = DEG2RAD(121.0);                   /* 经度，rad */
    init.llh[2] = 10.0;                             /* 椭球高，m */
    init.pos_init_stddev_m = 5.0f;
    init.vel_init_stddev_mps = 1.0f;
    init.rpy_init_stddev_rad[0] = (float)DEG2RAD(5.0); /* roll */
    init.rpy_init_stddev_rad[1] = (float)DEG2RAD(5.0); /* pitch */
    init.rpy_init_stddev_rad[2] = (float)DEG2RAD(5.0); /* yaw */

    opt.auto_init = true;
    /* 超过这些 σ 的 GNSS 观测不用于滤波器启动 */
    opt.gnss_start_max_horizontal_pos_stddev_m = 15.0f;
    opt.gnss_start_max_vertical_pos_stddev_m = 20.0f;
    opt.gnss_start_max_horizontal_vel_stddev_mps = 2.0f;
    opt.gnss_start_max_vertical_vel_stddev_mps = 2.0f;

    if (ins_init(&g_ins, &init, &opt) != 0) {
        Debug_Printf("ins_init failed\r\n");
        g_inited = 0U;
        return;
    }
    g_inited = 1U;
    g_last_imu_ms = HAL_GetTick();
    Debug_Printf("INS+RTK fusion init OK (UART7 PE7 NMEA)\r\n");
}

/**
 * @brief 主循环轮询：解析 NMEA，约 100 Hz 组一包量测并 ins_update
 *
 * 节拍：距上次步进 < 10 ms 时，NMEA 已在本函数开头解析，但直接返回，
 * 本周期不读 IMU、也不把 GNSS 送进滤波器。因此新 GGA/RMC/HDT 要等到
 * 下一次 dt >= 10 ms 才会融合。
 *
 * 量测包先清零。未置 is_valid 的通道表示本步没有该观测：
 *   - acc/gyr：IMU 正常且 BMI088_Read 成功
 *   - gnss_pos：新的 GGA（pos_valid 且 gga_cnt 变化）
 *   - gnss_vel：新的 RMC
 *   - yaw：新的双天线 HDT/THS
 * strapdown_dt_sec 大于 50 ms 时钳到 10 ms，避免主循环卡顿造成单步预测过大。
 *
 * ins_is_ready 为真（库内要求最少历元和约 1.5 s 运行时间）后才刷新
 * ins_out 的经纬高、NED 速度和欧拉角。
 */
void INS_Fusion_Poll(void)
{
    uint32_t now;
    uint32_t dt_ms;
    ins_measurements_t m;
    BMI088_Sample_t imu;
    float sh, sv;           /* GNSS 水平/垂直位置 1σ，米 */
    double llh[3];          /* 滤波器读出：lat/lon 弧度，高 米 */
    float roll, pitch, yaw; /* 弧度 */

    if (g_inited == 0U) {
        return;
    }

    /* 始终先消化 DMA 里的 NMEA，状态位每圈都更新，不受 100 Hz 节拍限制 */
    RTK_Poll();
    ins_out.rtk_ok = rtk_fix.pos_valid;
    ins_out.gnss_quality = rtk_fix.quality;

    now = HAL_GetTick();
    dt_ms = now - g_last_imu_ms;
    if (dt_ms < 10U) {
        return;
    }
    g_last_imu_ms = now;

    memset(&m, 0, sizeof(m));
    m.timestamp = now_us();
    m.strapdown_dt_sec = (float)dt_ms * 0.001f;
    if (m.strapdown_dt_sec > 0.05f) {
        m.strapdown_dt_sec = 0.01f;
    }

    /* 比力 m/s^2，角速度 rad/s，坐标系与 BMI088 驱动一致 */
    if ((ins_out.imu_ok != 0U) && (BMI088_Read(&imu) == 0)) {
        m.acc.is_valid = true;
        m.acc.data[0] = imu.acc_mps2[0];
        m.acc.data[1] = imu.acc_mps2[1];
        m.acc.data[2] = imu.acc_mps2[2];
        m.gyr.is_valid = true;
        m.gyr.data[0] = imu.gyr_rps[0];
        m.gyr.data[1] = imu.gyr_rps[1];
        m.gyr.data[2] = imu.gyr_rps[2];
    }

    /*
     * GNSS 位置。Qll_ned 按 3x3 行优先存放，只填对角：
     *   [0]=北方差，[4]=东方差，[8]=地方差，单位 m^2。
     */
    if ((rtk_fix.pos_valid != 0U) && (rtk_fix.gga_cnt != g_last_gga)) {
        g_last_gga = rtk_fix.gga_cnt;
        rtk_sigma(rtk_fix.quality, rtk_fix.hdop, &sh, &sv);
        m.gnss_pos.is_valid = true;
        m.gnss_pos.llh[0] = DEG2RAD(rtk_fix.lat_deg);
        m.gnss_pos.llh[1] = DEG2RAD(rtk_fix.lon_deg);
        m.gnss_pos.llh[2] = rtk_fix.alt_m;
        m.gnss_pos.Qll_ned[0] = sh * sh;
        m.gnss_pos.Qll_ned[4] = sh * sh;
        m.gnss_pos.Qll_ned[8] = sv * sv;
    }

    /*
     * GNSS 速度，NED，m/s。方差：水平 0.25（σ=0.5 m/s），垂向 1.0（σ=1 m/s）。
     */
    if ((rtk_fix.vel_valid != 0U) && (rtk_fix.rmc_cnt != g_last_rmc)) {
        g_last_rmc = rtk_fix.rmc_cnt;
        m.gnss_vel.is_valid = true;
        m.gnss_vel.vel_ned[0] = rtk_fix.vel_ned[0];
        m.gnss_vel.vel_ned[1] = rtk_fix.vel_ned[1];
        m.gnss_vel.vel_ned[2] = rtk_fix.vel_ned[2];
        m.gnss_vel.Qll_ned[0] = 0.25f;
        m.gnss_vel.Qll_ned[4] = 0.25f;
        m.gnss_vel.Qll_ned[8] = 1.00f;
    }

    /* 双天线真北航向。观测标准差取 0.5°，与厘米级基线的典型精度同量级 */
    if ((rtk_fix.yaw_valid != 0U) && (rtk_fix.hdt_cnt != g_last_hdt)) {
        g_last_hdt = rtk_fix.hdt_cnt;
        m.yaw.is_valid = true;
        m.yaw.yaw_rad = (float)DEG2RAD(rtk_fix.yaw_deg);
        m.yaw.stddev_rad = (float)DEG2RAD(0.5);
    }

    /* 捷联预测 + 本步有效量测的 ESKF 更新 */
    ins_update(&g_ins, &m);

    ins_out.timestamp_sample_us = RTK_UnixNowUs();
    ins_out.ready = ins_is_ready(&g_ins) ? 1U : 0U;
    ins_fusion_refresh_nav(llh, &roll, &pitch, &yaw);
}
