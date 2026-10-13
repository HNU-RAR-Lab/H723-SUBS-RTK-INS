/**
  ******************************************************************************
  * @file    ins_fusion.h
  * @brief   BMI088 IMU 与 UART7 RTK NMEA 的松耦合导航接口
  *
  * 底层为 INSLIB 15 状态误差状态卡尔曼滤波（ESKF）。本模块只负责采样、
  * 组量测和读出结果，不实现滤波公式。
  *
  * 调用约定：
  *   - 上电后调用一次 INS_Fusion_Init()
  *   - 主循环中反复调用 INS_Fusion_Poll()（内部将 IMU 步进限制在约 100 Hz）
  *   - 结果读全局 ins_out；ready==0 时经纬高/速度/姿态尚未可用
  ******************************************************************************
  */

#ifndef INS_FUSION_H
#define INS_FUSION_H

#include <stdint.h>

/**
 * @brief 融合输出，供其它任务或调试串口读取
 *
 * 位置为 WGS84 大地坐标；速度为当地北东地（NED）；姿态为机体相对 NED 的欧拉角。
 * 速度与姿态在滤波器未就绪（ready==0）时保持上次值，上电后为 0。
 */
typedef struct {
    uint8_t imu_ok;          /**< 1：BMI088 初始化成功；0：失败，后续不喂 IMU */
    uint8_t rtk_ok;          /**< 1：当前 rtk_fix 有有效 GNSS 位置（GGA） */
    uint8_t ready;           /**< 1：ins_is_ready() 为真，下面导航量可用 */
    double  lat_deg;         /**< 融合纬度，度，北纬为正 */
    double  lon_deg;         /**< 融合经度，度，东经为正 */
    double  h_m;             /**< 融合椭球高，米 */
    float   vel_ned[3];      /**< 北、东、地速度，m/s */
    float   roll_deg;        /**< 滚转角，度 */
    float   pitch_deg;       /**< 俯仰角，度 */
    float   yaw_deg;         /**< 航向角，度 */
    uint8_t gnss_quality;    /**< 最近 GGA quality：4=RTK 固定，5=RTK 浮点，2=差分 */
    uint64_t timestamp_us;          /**< 发布时间戳，Unix 微秒，与 PX4 timestamp 同格式 */
    uint64_t timestamp_sample_us;   /**< 采样时间戳，Unix 微秒 */
    uint8_t  xy_valid;              /**< 水平位置有效 */
    uint8_t  z_valid;               /**< 高度有效 */
    uint8_t  v_xy_valid;            /**< 水平速度有效 */
    uint8_t  v_z_valid;             /**< 垂直速度有效 */
    float    pos_ned[3];            /**< 相对原点北东地位置，米，z 向下为正 */
    float    acc_ned[3];            /**< 北东地加速度，m/s^2（不含重力） */
    float    heading_deg;           /**< 航向，度，[-180,+180]，0 为北 */
    float    heading_var;           /**< 航向方差，rad^2 */
    uint8_t  heading_good_for_control;
    double   ref_lat_deg;           /**< 局部原点纬度 */
    double   ref_lon_deg;
    double   ref_alt_m;
    float    eph;                   /**< 水平位置 1σ，米 */
    float    epv;                   /**< 垂直位置 1σ，米 */
    float    evh;                   /**< 水平速度 1σ，m/s */
    float    evv;                   /**< 垂直速度 1σ，m/s */
    uint8_t  dead_reckoning;        /**< 1：GNSS 中断后的推算航行 */
} INS_FusionOut_t;

extern INS_FusionOut_t ins_out;

/** @brief 初始化 BMI088、UART7 RTK 与 INSLIB。失败时 ins_out.imu_ok 或后续 Poll 会反映。 */
void INS_Fusion_Init(void);

/**
 * @brief 轮询 RTK、按约 100 Hz 读 IMU 并调用 ins_update，刷新 ins_out。
 * @note  距上次 IMU 步进不足 10 ms 时只解析 NMEA 后返回，不跑滤波。
 */
void INS_Fusion_Poll(void);

/** @brief USART10 以中文标识打印当前 RTK+IMU 融合结果（约 10 Hz 由主循环调用） */
void INS_Fusion_DebugPrint(void);

#endif
