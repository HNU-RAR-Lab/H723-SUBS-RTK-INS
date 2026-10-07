/**
  ******************************************************************************
  * @file    bmi088.h
  * @brief   达妙 DM-MC02 板载 BMI088（SPI2，软件片选）
  ******************************************************************************
  */

#ifndef BMI088_H
#define BMI088_H

#include <stdint.h>
#include "stm32h7xx_hal.h"

#define ACC_CS_Pin          GPIO_PIN_0
#define ACC_CS_GPIO_Port    GPIOC
#define GYRO_CS_Pin         GPIO_PIN_3
#define GYRO_CS_GPIO_Port   GPIOC

typedef struct {
    float acc_mps2[3];   /* 比力，机体轴 [m/s^2] */
    float gyr_rps[3];    /* 角速度 [rad/s] */
    float temp_c;
    uint8_t acc_id;
    uint8_t gyro_id;
    uint8_t ok;
} BMI088_Sample_t;

extern SPI_HandleTypeDef hspi2;
extern BMI088_Sample_t bmi088;

void BMI088_SPI_Init(void);
int  BMI088_Init(void);
int  BMI088_Read(BMI088_Sample_t *out);

#endif
