/**
  ******************************************************************************
  * @file    bmi088.c
  * @brief   DM-MC02 板载 BMI088：SPI2 PC1/MOSI、PC2/MISO、PB13/SCK
  *          ACC_CS=PC0，GYRO_CS=PC3，Mode3，轮询读写
  ******************************************************************************
  */

#include "bmi088.h"
#include "main.h"
#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

SPI_HandleTypeDef hspi2;
BMI088_Sample_t bmi088;

#define BMI088_ACC_CHIP_ID_REG   0x00U
#define BMI088_ACC_CHIP_ID_VAL   0x1EU
#define BMI088_ACC_DATA          0x12U
#define BMI088_ACC_TEMP_MSB      0x22U
#define BMI088_ACC_CONF          0x40U
#define BMI088_ACC_RANGE         0x41U
#define BMI088_ACC_PWR_CONF      0x7CU
#define BMI088_ACC_PWR_CTRL      0x7DU
#define BMI088_ACC_SOFTRESET     0x7EU

#define BMI088_GYRO_CHIP_ID_REG  0x00U
#define BMI088_GYRO_CHIP_ID_VAL  0x0FU
#define BMI088_GYRO_DATA         0x02U
#define BMI088_GYRO_RANGE        0x0FU
#define BMI088_GYRO_BANDWIDTH    0x10U
#define BMI088_GYRO_LPM1         0x11U
#define BMI088_GYRO_SOFTRESET    0x14U

/* 24g：1365 LSB/g；2000 dps：16.384 LSB/(deg/s) */
#define BMI088_ACC_LSB_PER_G     1365.0f
#define BMI088_GYRO_LSB_PER_DPS  16.384f

static void cs_acc(uint8_t level)
{
    HAL_GPIO_WritePin(ACC_CS_GPIO_Port, ACC_CS_Pin, level ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void cs_gyro(uint8_t level)
{
    HAL_GPIO_WritePin(GYRO_CS_GPIO_Port, GYRO_CS_Pin, level ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static HAL_StatusTypeDef spi_xfer(uint8_t *tx, uint8_t *rx, uint16_t n)
{
    return HAL_SPI_TransmitReceive(&hspi2, tx, rx, n, 5);
}

static void acc_write(uint8_t reg, uint8_t val)
{
    uint8_t tx[2];
    uint8_t rx[2];
    tx[0] = (uint8_t)(reg & 0x7FU);
    tx[1] = val;
    cs_acc(0);
    (void)spi_xfer(tx, rx, 2);
    cs_acc(1);
}

static uint8_t acc_read_u8(uint8_t reg)
{
    uint8_t tx[3] = {0};
    uint8_t rx[3] = {0};
    tx[0] = (uint8_t)(reg | 0x80U);
    cs_acc(0);
    (void)spi_xfer(tx, rx, 3); /* 地址 + dummy + 数据 */
    cs_acc(1);
    return rx[2];
}

static void acc_read_n(uint8_t reg, uint8_t *dst, uint8_t n)
{
    uint8_t tx[16];
    uint8_t rx[16];
    uint8_t i;
    memset(tx, 0, sizeof(tx));
    tx[0] = (uint8_t)(reg | 0x80U);
    cs_acc(0);
    (void)spi_xfer(tx, rx, (uint16_t)(n + 2U));
    cs_acc(1);
    for (i = 0; i < n; i++) {
        dst[i] = rx[i + 2U];
    }
}

static void gyro_write(uint8_t reg, uint8_t val)
{
    uint8_t tx[2];
    uint8_t rx[2];
    tx[0] = (uint8_t)(reg & 0x7FU);
    tx[1] = val;
    cs_gyro(0);
    (void)spi_xfer(tx, rx, 2);
    cs_gyro(1);
}

static uint8_t gyro_read_u8(uint8_t reg)
{
    uint8_t tx[2] = {0};
    uint8_t rx[2] = {0};
    tx[0] = (uint8_t)(reg | 0x80U);
    cs_gyro(0);
    (void)spi_xfer(tx, rx, 2);
    cs_gyro(1);
    return rx[1];
}

static void gyro_read_n(uint8_t reg, uint8_t *dst, uint8_t n)
{
    uint8_t tx[16];
    uint8_t rx[16];
    uint8_t i;
    memset(tx, 0, sizeof(tx));
    tx[0] = (uint8_t)(reg | 0x80U);
    cs_gyro(0);
    (void)spi_xfer(tx, rx, (uint16_t)(n + 1U));
    cs_gyro(1);
    for (i = 0; i < n; i++) {
        dst[i] = rx[i + 1U];
    }
}

void HAL_SPI_MspInit(SPI_HandleTypeDef *spi)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

    if (spi->Instance != SPI2) {
        return;
    }

    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SPI2;
    PeriphClkInitStruct.Spi123ClockSelection = RCC_SPI123CLKSOURCE_PLL;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK) {
        Error_Handler();
    }

    __HAL_RCC_SPI2_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    /* PC1=MOSI, PC2=MISO, PB13=SCK */
    GPIO_InitStruct.Pin = GPIO_PIN_1 | GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_13;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

void BMI088_SPI_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    GPIO_InitStruct.Pin = ACC_CS_Pin | GYRO_CS_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
    cs_acc(1);
    cs_gyro(1);

    hspi2.Instance = SPI2;
    hspi2.Init.Mode = SPI_MODE_MASTER;
    hspi2.Init.Direction = SPI_DIRECTION_2LINES;
    hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi2.Init.CLKPolarity = SPI_POLARITY_HIGH;
    hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
    hspi2.Init.NSS = SPI_NSS_SOFT;
    hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
    hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi2.Init.CRCPolynomial = 0x0;
    hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
    hspi2.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
    hspi2.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
    hspi2.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
    hspi2.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
    hspi2.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
    hspi2.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
    hspi2.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
    hspi2.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
    hspi2.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_ENABLE;
    hspi2.Init.IOSwap = SPI_IO_SWAP_DISABLE;
    if (HAL_SPI_Init(&hspi2) != HAL_OK) {
        Error_Handler();
    }
}

int BMI088_Init(void)
{
    BMI088_SPI_Init();
    HAL_Delay(2);

    /* 加速度计上电默认 I2C：拉一次 CS 上升沿切到 SPI */
    (void)acc_read_u8(BMI088_ACC_CHIP_ID_REG);
    HAL_Delay(1);

    acc_write(BMI088_ACC_SOFTRESET, 0xB6U);
    HAL_Delay(50);
    (void)acc_read_u8(BMI088_ACC_CHIP_ID_REG);
    HAL_Delay(1);

    gyro_write(BMI088_GYRO_SOFTRESET, 0xB6U);
    HAL_Delay(50);

    bmi088.acc_id = acc_read_u8(BMI088_ACC_CHIP_ID_REG);
    bmi088.gyro_id = gyro_read_u8(BMI088_GYRO_CHIP_ID_REG);

    acc_write(BMI088_ACC_PWR_CONF, 0x00U); /* active */
    HAL_Delay(1);
    acc_write(BMI088_ACC_PWR_CTRL, 0x04U); /* acc on */
    HAL_Delay(50);
    acc_write(BMI088_ACC_CONF, 0xA8U);     /* 100 Hz, normal BW */
    acc_write(BMI088_ACC_RANGE, 0x03U);    /* ±24 g */

    gyro_write(BMI088_GYRO_LPM1, 0x00U);
    HAL_Delay(1);
    gyro_write(BMI088_GYRO_RANGE, 0x00U);      /* ±2000 dps */
    gyro_write(BMI088_GYRO_BANDWIDTH, 0x07U);  /* 100 Hz */

    bmi088.ok = ((bmi088.acc_id == BMI088_ACC_CHIP_ID_VAL) &&
                 (bmi088.gyro_id == BMI088_GYRO_CHIP_ID_VAL)) ? 1U : 0U;
    return bmi088.ok ? 0 : -1;
}

int BMI088_Read(BMI088_Sample_t *out)
{
    uint8_t raw[6];
    int16_t ax, ay, az, gx, gy, gz;
    uint8_t tbuf[2];
    int16_t temp_raw;
    const float g = 9.80665f;
    const float dps_to_rps = (float)(M_PI / 180.0);

    if ((out == NULL) || (bmi088.ok == 0U)) {
        return -1;
    }

    acc_read_n(BMI088_ACC_DATA, raw, 6);
    ax = (int16_t)((uint16_t)raw[0] | ((uint16_t)raw[1] << 8));
    ay = (int16_t)((uint16_t)raw[2] | ((uint16_t)raw[3] << 8));
    az = (int16_t)((uint16_t)raw[4] | ((uint16_t)raw[5] << 8));

    gyro_read_n(BMI088_GYRO_DATA, raw, 6);
    gx = (int16_t)((uint16_t)raw[0] | ((uint16_t)raw[1] << 8));
    gy = (int16_t)((uint16_t)raw[2] | ((uint16_t)raw[3] << 8));
    gz = (int16_t)((uint16_t)raw[4] | ((uint16_t)raw[5] << 8));

    acc_read_n(BMI088_ACC_TEMP_MSB, tbuf, 2);
    temp_raw = (int16_t)(((uint16_t)tbuf[0] << 3) | ((uint16_t)tbuf[1] >> 5));
    if (temp_raw > 1023) {
        temp_raw -= 2048;
    }

    /* 板坐标系按丝印：X 前、Y 右、Z 下（FRD）。若安装相反，改这里的符号。 */
    out->acc_mps2[0] = ((float)ax / BMI088_ACC_LSB_PER_G) * g;
    out->acc_mps2[1] = ((float)ay / BMI088_ACC_LSB_PER_G) * g;
    out->acc_mps2[2] = ((float)az / BMI088_ACC_LSB_PER_G) * g;
    out->gyr_rps[0]  = ((float)gx / BMI088_GYRO_LSB_PER_DPS) * dps_to_rps;
    out->gyr_rps[1]  = ((float)gy / BMI088_GYRO_LSB_PER_DPS) * dps_to_rps;
    out->gyr_rps[2]  = ((float)gz / BMI088_GYRO_LSB_PER_DPS) * dps_to_rps;
    out->temp_c      = ((float)temp_raw * 0.125f) + 23.0f;
    out->ok          = 1U;
    out->acc_id      = bmi088.acc_id;
    out->gyro_id     = bmi088.gyro_id;
    bmi088 = *out;
    return 0;
}
