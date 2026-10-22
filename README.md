# H723-SUBS-RTK-INS

基于 **STM32H723VGT6** 与 **达妙 DM-MC02** 开发板的固件：接收 **Radiolink R9DS** 的 SBUS 遥控、输出两路推进器 PWM，并融合板载 **BMI088** 与 **UART7 RTK** NMEA 导航数据。

Keil 工程路径：`SUBS/MDK-ARM/SUBS.uvprojx`  
CubeMX 配置：`SUBS/SUBS.ioc`  
更详细的接线与调试说明见：`SUBS/达妙DM-MC02_R9DS_SBUS工程说明.txt`

---

## 系统时钟

| 项目 | 参数 |
|------|------|
| HSE | 24 MHz（板载晶振，**必须为 24 MHz**，否则 SBUS 100000 波特率不准） |
| SYSCLK | 200 MHz |
| APB1 / TIM2 | 定时器时钟 200 MHz（APB1 分频 ×2） |

---

## 引脚与功能总览

| 引脚 | 外设 | 方向 | 参数 / 说明 |
|------|------|------|-------------|
| **PD2** | UART5_RX | 输入 | **SBUS 接收**，100000，8E2，仅 RX；接板载 SUBS/SBUS 座（板载已硬件反相，软件 **关闭 RXINV**） |
| PC12 | UART5_TX | 输出 | 板级已连接，本工程 **不发送** |
| **PE3** | USART10_TX | 输出 | **调试串口**，115200 8N1 → USB-TTL RX，GND 共地 |
| PE2 | USART10_RX | 输入 | 可选，本工程未使用 |
| **PE7** | UART7_RX | 输入 | **RTK NMEA 接收**，115200 8N1，DMA 循环；RTK 模块 TX → PE7 |
| **PE8** | UART7_TX | 输出 | 板级四线座，本工程以接收为主 |
| **PA0** | TIM2_CH1 | 输出 | **推进器 PWM 1**，50 Hz，脉宽 1.0～2.0 ms，对应 SBUS `channels_norm[0]`（右摇杆） |
| **PA2** | TIM2_CH3 | 输出 | **推进器 PWM 2**，50 Hz，脉宽 1.0～2.0 ms，对应 SBUS `channels_norm[3]`（左摇杆） |
| **PC0** | GPIO | 输出 | BMI088 **加速度计片选** ACC_CS |
| **PC3** | GPIO | 输出 | BMI088 **陀螺仪片选** GYRO_CS |
| **PC1** | SPI2_MOSI | 输出 | BMI088 SPI 数据 |
| **PC2** | SPI2_MISO | 输入 | BMI088 SPI 数据 |
| **PB13** | SPI2_SCK | 输出 | BMI088 SPI 时钟 |
| **PA13** | SWDIO | — | 程序下载 / 调试（勿占用） |
| **PA14** | SWCLK | — | 程序下载 / 调试（勿占用） |
| PH0 / PH1 | HSE | — | 24 MHz 外部晶振 |

DMA（与引脚相关）：

| DMA | 用途 |
|-----|------|
| DMA1 Stream0 | UART5 RX → SBUS 循环缓冲（`sbus_dma_buf`，放在 AXI SRAM `RAM_D1`） |
| （RTK 模块内配置） | UART7 RX → RTK NMEA 循环缓冲 |

---

## SBUS 遥控通道（软件侧）

| 索引 | 典型用途 | 本工程行为 |
|------|----------|------------|
| `[0]` | 右摇杆左/右 | 手动模式下 → **PA0** 推进器脉宽 |
| `[3]` | 左摇杆左/右 | 手动模式下 → **PA2** 推进器脉宽 |
| `[8]` | **两档**拨杆（上拨约 **-1**，下拨约 **+1**） | **下拨 / 正值**：手动控制，PWM 跟随 `[0]`、`[3]`；**上拨 / 负值**：程序控制档（当前固件在该档不跟随摇杆，两路保持 **1.5 ms** 停转，程序控制逻辑预留在 `pwm.c` → `PWM_ApplyProgram()`） |
| 其余 | — | 解析并可在调试串口打印，**不驱动 PWM** |

归一化 `channels_norm`：中位约 **0**，摇杆两端约 **-1.0 / +1.0**。

---

## 推进器 PWM（电调 / ESC 脉宽）

| 项目 | 说明 |
|------|------|
| 频率 | **50 Hz**（周期 **20 ms**） |
| 定时器 | **TIM2**，PSC=199，ARR=19999 → 计数 **1 MHz**（1 计数 = **1 µs**） |
| 映射（手动模式，`channels_norm[8] > 0`） | **-1 → 1000 µs**（反转满量程）<br>**0 → 1500 µs**（停转）<br>**+1 → 2000 µs**（正转满量程）<br>中间 **线性**：`脉宽 = 1500 + norm × 500`（µs） |
| 上电默认 | 两路 **1500 µs**（停转） |
| 信号电平 | MCU 引脚 **3.3 V** 逻辑 PWM；若电调要求 5 V，需电平转换 |

方向由电调根据 **1.5 ms 中位** 解释：小于 1.5 ms 一侧为反转，大于 1.5 ms 一侧为正转（与常见航模 ESC 一致）。

---

## 导航与传感器（与 PWM 并行运行）

| 模块 | 接口 | 功能 |
|------|------|------|
| **BMI088** | SPI2（PC0/PC1/PC2/PC3，PB13） | 100 Hz 量级 IMU 采样，供 INSLIB ESKF |
| **RTK GNSS** | UART7（PE7/PE8），115200，NMEA | GGA 等解析，与 IMU 松耦合融合 |
| **INS_Fusion** | 软件 | `INS_Fusion_Init()` / `INS_Fusion_Poll()`；结果经 **USART10** 打印（含 RTK 双天线真北航向等） |

---

## 主程序数据流（简要）

```
R9DS SBUS → UART5/DMA → SBUS 解析 → channels_norm[0/3/8…]
                              ↓
                    PWM_SyncFromSbus() → TIM2 → PA0 / PA2

BMI088 (SPI2) ─┐
               ├→ INS_Fusion_Poll() → USART10 调试输出
RTK (UART7) ───┘
```

SBUS 解帧与写 CCR 在同一主循环内连续执行，并记录 DWT 微秒级时间戳（见 `pwm_sync`）。

---

## 编译与烧录

1. 用 **Keil MDK-ARM** 打开 `SUBS/MDK-ARM/SUBS.uvprojx`。
2. 确认 HSE = **24 MHz**（`SUBS.ioc` / `SystemClock_Config`）。
3. 编译、Download，**SWD**（PA13/PA14）连接调试器。

---

## 仓库结构

| 路径 | 说明 |
|------|------|
| `SUBS/Core/` | 应用代码（`main.c`、`sbus.c`、`pwm.c`、`ins_fusion.c`、`rtk_uart.c`、`bmi088.c` 等） |
| `SUBS/Drivers/` | STM32H7 HAL / CMSIS |
| `SUBS/Middlewares/INSLIB/` | 导航滤波库 |
| `SUBS/SUBS.ioc` | STM32CubeMX 工程配置 |

---

## 参考

- 板级资料：[达妙 DM-MC02（Gitee）](https://gitee.com/kit-miao/dm-mc02)
- 组织仓库：`HNU-RAR-Lab/H723-SUBS-RTK-INS`
