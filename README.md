# STM32H750 LHA7878 8 通道 24 位 ADC 采集工程

本项目基于 **STM32H750VBT6**，集成适配 **Legend Semi LHA7878**（8 通道 24 位同步采样 Delta-Sigma ADC）完善版驱动，实现 8 通道高精度数据高速连续采集（8kSPS）、补码转微伏（µV）换算与串口实时打印。

## 硬件连接与引脚定义

| LHA7878 引脚 | STM32H750 引脚 | CubeMX 标签 / 宏名 | 功能说明 |
|:---|:---|:---|:---|
| **SCLK** | PA9 | SPI2_SCK | 硬件 SPI2 时钟（18.75 MHz） |
| **MISO (DOUT)** | PC2_C | SPI2_MISO | 硬件 SPI2 数据输入 |
| **MOSI (DIN)** | PC3_C | SPI2_MOSI | 硬件 SPI2 数据输出（读数时保持低电平） |
| **CS#** | PC8 | SPI2_CS | 软件片选（低有效） |
| **START** | PB10 | START | 启动转换（高电平启动） |
| **DRDY#** | PB0 | DRDY# | 数据就绪指示（低有效） |
| **PWDN#** | PB6 | PWDN# | 掉电控制（低有效，工作时拉高） |
| **RESET#** | PB7 | LHA7878_nRESET | 硬件复位（低有效，工作时拉高） |
| **USART2 TX** | PA2 | USART2_TX | 调试串口输出（波特率 115200） |
| **USART2 RX** | PA3 | USART2_RX | 调试串口输入 |
| **LED** | PE3 | LED | 运行指示灯（每 200ms 翻转） |

## 驱动与外设配置要点

1. **SPI 配置**：
   - 模式：**SPI Mode 1**（`CPOL = Low`, `CPHA = 2 Edge`）
   - 数据位：8-bit，MSB First，软件 NSS
   - SCLK 波特率：由 75MHz PLL 时钟 4 分频得出 18.75 MBits/s（符合 3.3V DVDD 电气规范）
2. **驱动特性（依据 LHA787X 中文 Rev.D 手册核对）**：
   - 寄存器表按 Rev.D 重构，严格支持回读校验；
   - 字节间插 `tSDECODE` 延时（基于 DWT 计数器精确微秒延时）；
   - 读数时 DIN 严格保持低电平（发送 0x00，而非 0xFF）；
   - 24 位补码换算采用 64 位整型公式：`V = code * 2 * VREF / Gain / 2^24`，杜绝溢出；
   - 状态字校验前 4 位同步码 `1100b`，并解析 `FAULT_P` / `FAULT_N`。

## 构建与运行

- **IDE / 编译器**：Keil MDK (uVision 5.32+), 编译器采用 **ArmClang (AC6)**
- **编译状态**：0 Error(s), 0 Warning(s)
- **串口波特率**：115200 bps, 8-N-1
- **串口发送**：USART2 TX 经 DMA1 Stream7 发送（`main.c` 内手工初始化：行缓冲攒满整行后一次 DMA 发出，发送完成由 USART2 TC 中断确认；DMA 初始化失败时自动回落轮询发送）。**CubeMX 侧不要再勾选 USART2 的 DMA**，否则与手工初始化冲突（链接报 `DMA1_Stream7_IRQHandler` 重定义）；若改由 CubeMX 接管，需先删除 `main.c` 里的手工初始化与两个中断服务函数。
- **输出格式**：
  ```text
  ========================================
    STM32H750 LHA7878 8-Channel ADC Demo
  ========================================
  [INIT] Starting LHA7878 initialization...
  [INIT] LHA7878 initialized successfully! (REV_ID OK)
  [INIT] Entering continuous 8-channel acquisition...

  [Frame #1200] Approx Rate: 8000 sps | FaultP=0x00 FaultN=0x00
    CH1:   +123456 uV ( +123.456 mV) | CH2:    -65432 uV (  -65.432 mV)
    ...
  ```
