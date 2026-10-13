/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : 工程入口：初始化时钟/UART/DMA，启动 SBUS 接收并循环解析打印
  *
 * 达妙 DM-MC02 + Radiolink R9DS：
 *   UART5  PD2  接收 SBUS（100000，8E2，板载硬件反相，软件 RXINV 关）
 *   USART10 PE3 调试打印到电脑（115200，8N1）
 *   TIM2   PA0/PA2  50Hz PWM，对应 channels_norm[0]/[3]
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "sbus.h"   /* SBUS 解析与调试打印 */
#include "pwm.h"    /* TIM2 两路 50Hz PWM 与时间戳同步 */
#include "ins_fusion.h" /* BMI088 + RTK 松耦合导航 */
#include "rtk_uart.h"   /* UART7 DMA 出错后重启 */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

TIM_HandleTypeDef htim2;         /* TIM2 句柄：PA0/PA2 两路 50Hz PWM */
UART_HandleTypeDef huart5;       /* UART5 句柄：SBUS 接收，引脚 PD2=RX */
UART_HandleTypeDef huart10;      /* USART10 句柄：调试打印，引脚 PE3=TX */
DMA_HandleTypeDef hdma_uart5_rx; /* UART5 接收 DMA（DMA1 Stream0，循环模式） */

/* USER CODE BEGIN PV */
/* DMA 循环接收缓冲。volatile 防止 -O3 把数组当常量；section RAM_D1 放到 AXI SRAM
 * （0x24000000），因为 H7 的 DMA1 无法写入 DTCM（0x20000000）。 */
#if defined(__CC_ARM)
volatile uint8_t sbus_dma_buf[SBUS_DMA_BUF_LEN] __attribute__((section("RAM_D1"), zero_init, aligned(32)));
#else
volatile uint8_t sbus_dma_buf[SBUS_DMA_BUF_LEN] __attribute__((section("RAM_D1"), aligned(32)));
#endif
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);          /* 配置 HSE 24MHz + PLL，SYSCLK=200MHz */
static void MPU_Config(void);           /* 配置 MPU，禁止对未实现地址的投机访问 */
static void MX_GPIO_Init(void);         /* 打开本工程用到的 GPIO 时钟 */
static void MX_DMA_Init(void);          /* 打开 DMA1 时钟并允许 DMA1 Stream0 中断 */
static void MX_USART10_UART_Init(void); /* 初始化调试串口 USART10 */
static void MX_UART5_Init(void);        /* 初始化 SBUS 串口 UART5 */
static void MX_TIM2_Init(void);         /* 初始化 TIM2 两路 PWM（50Hz） */
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim); /* CubeMX：PWM 引脚复用 */
/* USER CODE BEGIN PFP */
static void SBUS_StartDmaRx(void);      /* 启动 UART5 DMA 收数并打开空闲中断 */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  程序入口：MPU → HAL → 时钟 → 外设 → 启动 DMA 接收 → 死循环轮询解析
  * @retval int  正常情况不会返回
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART10_UART_Init();
  MX_UART5_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  PWM_Init();
  /* 启动时向电脑打印一行，确认 USART10（PE3）串口正常 */
  Debug_Printf("SBUS+PWM init OK\r\n");

  /* 板载 BMI088 + UART7 RTK NMEA → INSLIB ESKF，不改变原有 SBUS/PWM */
  INS_Fusion_Init();

  /* 启动 UART5 DMA 接收 + 空闲中断
   * 当 SBUS 一帧发送完毕、线路空闲时，触发 HAL_UARTEx_RxEventCallback */
  SBUS_StartDmaRx();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    static uint32_t last_print = 0; /* 上次串口打印的 HAL 时基，单位 ms */

    /* 主循环轮询 DMA。解帧只在这里做，不依赖空闲中断 */
    SBUS_PollFromUart(&huart5, sbus_dma_buf, SBUS_DMA_BUF_LEN);
    /* 新帧到达后立刻改 PWM，使 SBUS 接收时间戳与 PWM 输出时间戳对齐 */
    PWM_SyncFromSbus();
    /* IMU 100Hz + RTK NMEA 融合，与遥控互不影响 */
    INS_Fusion_Poll();

    /* 每 100ms（10Hz）打印一次，避免刷屏过快 */
    if ((HAL_GetTick() - last_print) >= 100U) {
      last_print = HAL_GetTick();

      /* 主循环解析成功后 sbus_data.updated 会被置 1 */
      if (sbus_data.updated) {
        sbus_data.updated = 0;
        /* channels_norm：中位 0.0，摇杆两端约 -1.0 / +1.0
         * FL=丢帧标志，FS=失控保护（遥控器断电或超出距离时为 1）
         * [0]：右操纵杆左/右，[3]：左操纵杆左/右 */
        Debug_Printf(
            "N1:%+.2f N4:%+.2f PWM:%lu/%lu "
            "RX:%lu PWM_T:%lu dT:%ld FL:%u FS:%u\r\n",
            sbus_data.channels_norm[0],
            sbus_data.channels_norm[3],
            pwm_sync.pulse[0], pwm_sync.pulse[1],
            pwm_sync.sbus_rx_us, pwm_sync.pwm_out_us, pwm_sync.dt_us,
            sbus_data.frame_lost, sbus_data.failsafe);
      }
      /* UART10：RTK+IMU 融合结果，中文标识，时间戳为 Unix 微秒 */
      INS_Fusion_DebugPrint();
    }
  }
  /* USER CODE END 3 */
}

/**
  * @brief  系统时钟：板载 24MHz HSE，PLL M=6 N=100 P=2 → SYSCLK 200MHz，APB 各 100MHz
  * @note   HSE 必须填 24MHz，误填 25MHz 会导致 100000 波特率偏差，SBUS 解不开
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0}; /* 振荡器/PLL 参数 */
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0}; /* SYSCLK/AHB/APB 分频 */

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief  TIM2 两路 PWM：内部时钟，50Hz
  * @note   APB1 分频后定时器时钟为 200MHz；PSC=199 → 1MHz；ARR=19999 → 20ms
  *         PA0=CH1←SBUS[0]，PA2=CH3←SBUS[3]
  *         初始 Pulse=10000 为 50% 占空比
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = PWM_TIM_PRESCALER;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = PWM_TIM_PERIOD;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = PWM_PULSE_50_PERCENT;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief  初始化 UART5 为 SBUS 接收口
  * @note   波特率 100000；STM32 字长含校验位，故 9B+Even=8 数据位+偶校验；2 停止位；仅接收；
  *         板载 SUBS 已硬件反相，软件 RXINV 关闭。引脚 PD2=RX、PC12=TX。
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 100000;
  /* STM32 字长包含校验位：9B + Even = 8 数据位 + 偶校验（SBUS 标准 8E2） */
  huart5.Init.WordLength = UART_WORDLENGTH_9B;
  huart5.Init.StopBits = UART_STOPBITS_2;
  huart5.Init.Parity = UART_PARITY_EVEN;
  huart5.Init.Mode = UART_MODE_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  huart5.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart5.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  /* DM-MC02 板载 SUBS 座前已有硬件反相，PD2 上为标准 UART，软件不再开 RXINV */
  huart5.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart5, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart5, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */
  CLEAR_BIT(huart5.Instance->CR2, USART_CR2_RXINV);
  /* USER CODE END UART5_Init 2 */

}

/**
  * @brief  初始化 USART10 为电脑调试口：115200 8N1，PE3=TX，不反相、不开 DMA/中断
  */
static void MX_USART10_UART_Init(void)
{

  /* USER CODE BEGIN USART10_Init 0 */

  /* USER CODE END USART10_Init 0 */

  /* USER CODE BEGIN USART10_Init 1 */

  /* USER CODE END USART10_Init 1 */
  huart10.Instance = USART10;
  huart10.Init.BaudRate = 115200;
  huart10.Init.WordLength = UART_WORDLENGTH_8B;
  huart10.Init.StopBits = UART_STOPBITS_1;
  huart10.Init.Parity = UART_PARITY_NONE;
  huart10.Init.Mode = UART_MODE_TX; /* 调试口只发，不收；PE2 浮空时 RX 会卡死发送 */
  huart10.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart10.Init.OverSampling = UART_OVERSAMPLING_16;
  huart10.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart10.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart10.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart10) != HAL_OK)
  {
    Error_Handler();
  }
  /* FIFO 开关失败不得进 Error_Handler，否则调试口永远无输出 */
  (void)HAL_UARTEx_DisableFifoMode(&huart10);
  /* USER CODE BEGIN USART10_Init 2 */
  CLEAR_BIT(huart10.Instance->CR1, USART_CR1_RE);
  SET_BIT(huart10.Instance->CR1, USART_CR1_UE | USART_CR1_TE);
  __HAL_UART_CLEAR_FLAG(&huart10, UART_CLEAR_PEF | UART_CLEAR_FEF |
                        UART_CLEAR_NEF | UART_CLEAR_OREF);
  /* USER CODE END USART10_Init 2 */

}

/**
  * @brief  使能 DMA1 时钟，并打开 UART5 RX 所用的 DMA1 Stream0 中断
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream1：UART7 RX（RTK），不与 UART5 Stream0 冲突 */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);

}

/**
  * @brief  打开 GPIOE/H/A/C/D 时钟（UART、HSE、SWD、SPI 引脚所在端口）
  * @note   RTK 由模块 USB 独立供电，不打开板载 POWER_5V（PC15），
  *         UART7 座 VCC 不接模块。
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/**
  * @brief  启动 UART5 循环 DMA 接收，并打开空闲中断
  * @note   关闭 PE/FE 错误中断，避免 HAL 把噪声当阻塞错误把 DMA 杀掉。
  *         真正解帧由主循环 SBUS_PollFromUart 完成。
  */
static void SBUS_StartDmaRx(void)
{
  CLEAR_BIT(huart5.Instance->CR2, USART_CR2_RXINV);

  __HAL_UART_CLEAR_FLAG(&huart5, UART_CLEAR_IDLEF | UART_CLEAR_PEF |
                        UART_CLEAR_FEF | UART_CLEAR_NEF | UART_CLEAR_OREF);

  if (HAL_UARTEx_ReceiveToIdle_DMA(&huart5, (uint8_t *)sbus_dma_buf, SBUS_DMA_BUF_LEN) != HAL_OK) {
    Debug_Printf("SBUS DMA start failed\r\n");
    return;
  }

  /* SBUS 偶发噪声会置 PE/FE。HAL 在 DMA 模式下把它们当阻塞错误，
   * 会 abort 并直接 return，IDLE 回调永远进不去。关掉错误中断，解帧靠主循环轮询。 */
  ATOMIC_CLEAR_BIT(huart5.Instance->CR1, USART_CR1_PEIE);
  ATOMIC_CLEAR_BIT(huart5.Instance->CR3, USART_CR3_EIE);

  huart5.ReceptionType = HAL_UART_RECEPTION_TOIDLE;
  __HAL_UART_CLEAR_FLAG(&huart5, UART_CLEAR_IDLEF);
  ATOMIC_SET_BIT(huart5.Instance->CR1, USART_CR1_IDLEIE);
}

/**
  * @brief  UART 空闲/DMA 完成回调（HAL 弱函数覆盖）
  * @param  huart  触发该事件的 UART
  * @param  Size   HAL 给出的长度/位置，语义不稳定，本工程不解帧，仅计数
  */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  (void)Size; /* 明确不使用 Size，避免误当 DMA 写入下标 */
  if (huart->Instance == UART5) {
    /* 只统计中断。HAL 的 Size 不一定等于 DMA 写入下标，
     * 若在这里和主循环同时解帧，字节会错位，frame_ok_cnt 就不增加。 */
    sbus_dbg.rx_event_cnt++;
  }
}

/**
  * @brief  UART 错误回调：记录 ErrorCode 后重新启动 DMA 接收
  * @param  huart  出错的 UART，仅处理 UART5
  */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == UART5) {
    sbus_dbg.uart_err_cnt++;
    sbus_dbg.last_uart_err = huart->ErrorCode;
    SBUS_StartDmaRx();
  } else if (huart->Instance == UART7) {
    RTK_StartDmaRx();
  }
}
/* USER CODE END 4 */

 /* MPU Configuration */

/**
  * @brief  CubeMX 默认 MPU：屏蔽未实现地址空间的投机读，避免 HardFault
  */
void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0}; /* 单块 4GB 区域的 MPU 描述符 */

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  HAL 初始化失败时进入：关中断后死循环，便于 Debug 停在此处排查
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
