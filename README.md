# STM32H750 LHA7878 8 通道 24 位 ADC 采集工程

本项目基于 **STM32H750VBT6**，集成适配 **Legend Semi LHA7878**（8 通道 24 位同步采样 Delta-Sigma ADC）完善版驱动，实现 8 通道高精度数据高速连续采集（8 kSPS）、抗碰撞时序控制、累加均值滤波、DMA 串口发送与实时表格化打印。

## 硬件连接与引脚定义

| LHA7878 引脚 | STM32H750 引脚 | CubeMX 标签 / 宏名 | 功能说明 |
|:---|:---|:---|:---|
| **SCLK** | PA9 | SPI2_SCK | 硬件 SPI2 时钟（9.375 MHz，PLL1Q 75MHz / 8 分频） |
| **MISO (DOUT)** | PC2_C | SPI2_MISO | 硬件 SPI2 数据输入 |
| **MOSI (DIN)** | PC3_C | SPI2_MOSI | 硬件 SPI2 数据输出（读数时保持低电平） |
| **CS#** | PC8 | SPI2_CS | 软件片选（低有效） |
| **START** | PB10 | START | 启动转换（高电平启动） |
| **DRDY#** | PB0 | DRDY# | 数据就绪指示（低有效） |
| **PWDN#** | PB6 | PWDN# | 掉电控制（低有效，工作时拉高） |
| **RESET#** | PB7 | LHA7878_nRESET | 硬件复位（低有效，工作时拉高） |
| **CLKSEL (Pin 52)**| VDD (高电平) | - | 选用芯片内部 2.048 MHz 振荡器 |
| **USART2 TX** | PA2 | USART2_TX | 调试串口输出（波特率 1,500,000） |
| **USART2 RX** | PA3 | USART2_RX | 调试串口输入 |
| **LED** | PE3 | LED | 运行指示灯（每 200ms 翻转） |

## 关键技术要点与时序优化

1. **8 kSPS 速率锁定**：
   - 依据《LHA787X Datasheet CN Rev.D》§9.5.3.7，初始化阶段跳过 `OFFSETCAL` 指令（因硬件执行该指令会强制将数据速率改写为 1 kSPS，`DR[2:0]=110b`）。
   - 实测 DRDY 周期精准维持在 **124.45 µs**（频率 **8,035.4 Hz**，与内部 2.048 MHz 时钟 1:1 吻合）。
2. **SPI 高速移位与抗碰撞时序（Anti-Collision）**：
   - 将 SPI2 预分频配置为 8（~9.375 MHz），单帧 27 字节移位耗时由 ~64 µs 压缩至 **~23 µs**，为下一个 124.45 µs DRDY 周期留出 **>100 µs 安全裕量**。
   - `lha7878_wait_drdy()` 采用边沿捕获逻辑（先确保旧周期低电平释放，再捕获新周期的真正下降沿），彻底规避更新窗口（$t_{UPDATE}=4/f_{CLK}$）移位碰撞，坏帧率降至 **0.000%**。
3. **数据滤波与 DMA 串口输出**：
   - 采用 200ms 滑动窗口累加均值滤波（单次统计约 1,600 点采样），有效压低白噪声，输出极度平稳。
   - 调试串口以 **1,500,000 bps** 速率输出表格化数据，并实时展示实际采样速率（`actual_sps`）。
   - **串口 DMA 发送**：USART2 TX 经 DMA1 Stream7 发送（`main.c` 内手工初始化：行缓冲攒满整行后一次 DMA 发出，发送完成由 USART2 TC 中断确认；DMA 初始化失败时自动回落轮询发送）。**CubeMX 侧不要再勾选 USART2 的 DMA**，否则与手工初始化冲突（链接报 `DMA1_Stream7_IRQHandler` 重定义）；若改由 CubeMX 接管，需先删除 `main.c` 里的手工初始化与两个中断服务函数。

## 构建与运行

- **IDE / 编译器**：Keil MDK (uVision 5.32+), 编译器采用 **ArmClang (AC6)**
- **编译状态**：0 Error(s), 0 Warning(s)
- **串口波特率**：1,500,000 bps, 8-N-1
- **输出格式示例**：
  ```text
  ========================================
    STM32H750 LHA7878 8-Channel ADC Demo
  ========================================
  [INIT] Starting LHA7878 initialization...
  [INIT] LHA7878 initialized successfully! (REV_ID OK)
  [INIT] Entering continuous 8-channel acquisition...

    Rate: 8000 sps (Samples: 1600)
    Frame |    CH1    |    CH2    |    CH3    |    CH4    |    CH5    |    CH6    |    CH7    |    CH8    |  mV
   213300 |    +0.007 |    +0.000 |    +0.027 |    -0.008 |    +0.012 | +1476.709 |    -0.002 |    +0.003 |
  ```
