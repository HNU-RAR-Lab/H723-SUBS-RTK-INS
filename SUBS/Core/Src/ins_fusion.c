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

    ins_out.ready = ins_is_ready(&g_ins) ? 1U : 0U;
    if (ins_out.ready != 0U) {
        if (ins_get_latlonh(&g_ins, llh)) {
            ins_out.lat_deg = RAD2DEG(llh[0]);
            ins_out.lon_deg = RAD2DEG(llh[1]);
            ins_out.h_m = llh[2];
        }
        (void)ins_get_velocity_ned(&g_ins, ins_out.vel_ned);
        if (ins_get_rpy(&g_ins, &roll, &pitch, &yaw)) {
            ins_out.roll_deg = (float)RAD2DEG(roll);
            ins_out.pitch_deg = (float)RAD2DEG(pitch);
            ins_out.yaw_deg = (float)RAD2DEG(yaw);
        }
    }
}
