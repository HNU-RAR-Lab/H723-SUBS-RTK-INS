/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32h7xx_it.h
  * @brief   This file contains the headers of the interrupt handlers.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32H7xx_IT_H
#define __STM32H7xx_IT_H

#ifdef __cplusplus
extern "C" {
#endif

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void NMI_Handler(void);              /* 不可屏蔽中断 */
void HardFault_Handler(void);        /* 硬件错误，死循环便于 Debug */
void MemManage_Handler(void);        /* MPU 访问错误 */
void BusFault_Handler(void);         /* 总线错误 */
void UsageFault_Handler(void);       /* 用法错误（未定义指令等） */
void SVC_Handler(void);              /* SVCall，本工程未使用 */
void DebugMon_Handler(void);         /* 调试监视器 */
void PendSV_Handler(void);           /* 可挂起系统调用，本工程未使用 */
void SysTick_Handler(void);          /* 1ms 节拍，驱动 HAL_GetTick */
void DMA1_Stream0_IRQHandler(void);  /* UART5 RX DMA 中断 */
void UART5_IRQHandler(void);         /* UART5 空闲/错误中断 */
void DMA1_Stream1_IRQHandler(void);  /* UART7 RX DMA */
void UART7_IRQHandler(void);         /* UART7 空闲/错误中断 */
/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

#ifdef __cplusplus
}
#endif

#endif /* __STM32H7xx_IT_H */
