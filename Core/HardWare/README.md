# LHA7878 驱动（完善版）

依据：Legend Semi **LHA787X Datasheet CN Rev.D**（46 页，2025-10-21）。
核对章节：§7.3、§7.5、§7.6 表 8、§7.7 表 9、§9.3.2、§9.3.3、§9.3.4、
§9.3.7、§9.3.8、§9.3.9、§9.3.10、§9.4.1 表 15、§9.4.2、§9.4.3、
§9.4.5 图 34/35、§9.5、§9.5.3.1、§9.6 表 19/§9.6.1.1–§9.6.1.9、
§10.1.2、§10.1.3、§11.1 表 39、图 42。

## 文件

- `LHA7878.h` / `LHA7878.c`：完善版驱动，直接替换原版同名文件。
- 原版：`workspace/user/files/LHA7878.c` / `LHA7878.h`（已保留，不改动）。

## 原版驱动的问题清单（按手册逐项核对）

| # | 问题 | 手册依据 | 新版做法 |
|---|---|---|---|
| 1 | 寄存器表错：0x0D–0x11 是保留寄存器，0x15–0x17 不存在（LOFF/BIAS/MISC/CONFIG4 是另一颗芯片的表） | §9.6 表 19 | 按 Rev.D 重写，保留寄存器不单独配置 |
| 2 | `DUT_WTREG` 发 `0xA1/0x82…`，不是合法 WREG 帧；USERKEY UNLOCK / REF_PREQ 在 Rev.D 里不存在 | §9.5.3.12、§9.6 | 删除。如板子真需要未公开解锁流程，向 FAE 索要书面说明，不要写回 |
| 3 | `CONFIG1 0x18+DR` 本身没错，但 DAISY_IN=0 是菊花链模式；单片多次回读应为 1 | §9.6.1.2 | 配置项 `multi_readback`，默认 1 |
| 4 | `CONFIG3 0xA0` 写死 4V 基准，未校验 AVDD；AVDD=3V 时内部基准必须为 2.4V | §9.3.8 | `vref` 枚举 + `avdd_mv` 校验，4V 时 AVDD<4.5V 返回 ERR_VREF |
| 5 | `LHA7878_delay_us` 是未标定的 NOP 循环，5µs 延时不可信；tSDECODE 要求 1.96µs | §9.5.3.1、§7.6 | DWT CYCCNT 精确延时，可弱重定义 |
| 6 | 读数用 `HAL_SPI_Receive`，MOSI 发 0xFF；手册要求读数全程 DIN 保持低电平 | §9.4.5.2 | 一律 `TransmitReceive`，TX 全 0x00 |
| 7 | `RDATA()` 只发命令就拉高 CS，不读数据，数据丢了 | §9.5.3.10 | 同一 CS 周期内发 0x12 后紧接移 27 字节 |
| 8 | `RREG` 在 RDATAC 下会被忽略，原版 `read_register` 在 init 后（已进 RDATAC）调用必失败 | §9.5.3 表 18 注 1 | 读/写寄存器时若在 RDATAC 返回 ERR_ARG，使用方先 SDATAC |
| 9 | 换算写死 `data*2*5000/2^24`，把 AVDD 当 VREF，且不分增益 | §9.5.1 公式 (8) | `code*2*VREF/Gain/2^24`，int64 计算，分通道增益 |
| 10 | 状态字只判 `(buf[0]&0xC0)==0xC0`，且不解析 FAULT_STATP/N、GPIO | §9.4.5.3 图 35 | 判满 4 位 `1100b`，解析 fault_p/n、gpio，故障置位时返回 ERR_FAULT（帧仍保留） |
| 11 | 无返回值、无 ID 校验、无回读校验、无超时，`read_id` 返回 0xFF/0x00 无法区分错误 | — | 全部 API 返回 `lha7878_status_t`，init 回读校验 ID 与每个寄存器 |
| 12 | `#if 0` 里的 GPIO 模拟 SCLK 循环只发时钟不采 MISO，且 `while(1)` 在 init 里，若误开会挂死 | — | 删除，改用硬件 SPI + DRDY 轮询/中断 |
| 13 | 无 OFFSETCAL、无故障阈值、无 GPIO、无测试信号/温度/MVDD、无菊花链说明 | §9.5.3.7、§9.3.9、§9.3.10、§9.3.2 | 全部补上；多片菊花链明确不在本版实现范围内 |
| 14 | 128kSPS + RDATAC 读不完却无任何拦截 | §9.5.2.2 公式 9 | `init()` 返回 ERR_RATE，必须二选一（见下） |

原版写对了、已保留的部分：CHnSET 0x10（增益 1、正常输入）、
CONFIG1 基值 0x18 的保留位、RREG/WREG 的两字节命令格式、
27 字节帧长与通道偏移（状态字 3 字节 + CH1 在字节 3–5，依此类推）、
状态字高两位为 1 的帧同步思路（新版判满 4 位）。

## 128kSPS 二选一（未决，等用户点选）

- **A. 保持 128kSPS，改 RDATA**：`mode = LHA7878_MODE_RDATA`，
  `data_rate = LHA7878_DR_128K`，CONFIG1 = 0x1F。
  每帧先发 RDATA 0x12 再移 216 位，20MHz 下约 11.2µs，跨一个 DRDY，
  手册允许重叠，但取不完每一帧时会静默跳帧，须用帧序号核对。
- **B. 降到 64kSPS，保持 RDATAC**：`mode = LHA7878_MODE_RDATAC`，
  `data_rate = LHA7878_DR_64K`，CONFIG1 = 0x18。
  20MHz 下 216 位耗时 10.8µs，约剩 2.9µs 裕量；16MHz 下裕量仅约 0.17µs，
  所以用 20MHz。

减通道不能缩短帧（§9.4.5.4：关掉只填 0，帧恒 216 位），不要再选这条路。

## CubeMX 前置条件

- SPI2：CPOL=Low，CPHA=2 Edge（Mode 1），8-bit，MSB first，NSS=Software。
  配置阶段 SCLK 不必降速（驱动在 WREG/RREG 字节间插 tSDECODE 延时），
  读数 SCLK 按速率查 `lha7878_min_sclk_hz()`，实操建议 3.3V 时 10–20MHz。
- GPIO：CS（软件控制）、START、RESET（低有效）、PWDN（低有效）、
  DRDY（低有效，下降沿中断或轮询）。
- 引脚名在 `lha7878_dev_default()` 里按原工程填写，
  DRDY 按原版轮询代码推断为 PB0，若 CubeMX 里不是 PB0，只改这一处。

## 最小使用示例

```c
#include "LHA7878.h"

static lha7878_dev_t adc;
static lha7878_frame_t frame;
static int64_t uv[8];

void app_init(void)
{
    lha7878_config_t cfg;
    lha7878_config_default(&cfg);
    cfg.avdd_mv = 5000;              /* 按板子改: 3000 或 5000 */
    cfg.vref = LHA7878_VREF_INT_4V;  /* AVDD=5V 时才可这样配, 否则用 INT_2V4 */
    cfg.data_rate = LHA7878_DR_8K;
    cfg.mode = LHA7878_MODE_RDATAC;
    cfg.sclk_hz = 10000000;
    lha7878_dev_default(&adc, &cfg);

    lha7878_status_t st = lha7878_init(&adc);
    if (st != LHA7878_OK) {
        /* st 会告诉你是 ERR_ID / ERR_VERIFY / ERR_RATE / ERR_VREF 等, 不要忽略 */
        Error_Handler();
    }
}

void app_loop(void)
{
    /* RDATAC 模式: 等 DRDY, 读一帧, 换算. 超时时间按速率给, 如 8kSPS 给 5ms. */
    lha7878_status_t st = lha7878_read_frame(&adc, &frame, 5);
    if (st == LHA7878_OK || st == LHA7878_ERR_FAULT) {
        lha7878_frame_to_uv(&adc, &frame, uv);  /* uv[i] 单位 uV */
    }
    /* st == ERR_FRAME 时 frame_error_count 自增, 建议记数后排查接线/SCLK. */
}
```

128kSPS 选 A 时，把 `cfg.data_rate` 改为 `LHA7878_DR_128K`、
`cfg.mode` 改为 `LHA7878_MODE_RDATA` 即可，其余调用不变。

## 验证（2026-11-07，Linux gcc 桩 HAL）

- `gcc -Wall -Wextra -std=c99 -fsyntax-only`：驱动本身零警告。
- 逻辑测试全部通过：8kSPS 最低 SCLK 1.755MHz、64kSPS 15.80MHz、
  128kSPS 36.86MHz；128kSPS+RDATAC 返回 ERR_RATE，
  128kSPS+RDATA 与 64kSPS+RDATAC 返回 OK；
  状态字解析（FAULT_STATP/N、GPIO、符号扩展）与
  满量程换算（±8388607 → ±3999999µV @VREF 4V）均对。
- 未在实板上验证：上板后先跑 `lha7878_selftest_short()`（噪声）
  与 `lha7878_selftest_signal()`（内部测试信号），再接真实信号。

## 待用户确认（不要替用户定）

1. 128kSPS 的 A/B 二选一（见上）。
2. 板上 AVDD 是 3V 还是 5V、基准走内部 2.4V / 内部 4V / 外部，
   以及外部基准时的实际电压值（`vref_ext_uv`）。
3. DRDY 引脚是不是 PB0；CLK 是外部 2.048MHz 还是内部振荡器
   （`clk_hz` / `clk_en` / CLKSEL 引脚）。
4. `DUT_WTREG` 的 USERKEY/REF_PREQ 是不是另一颗芯片（或另一版
   手册）的流程——如果是，请贴出那份手册/来源，Rev.D 里没有。
5. 是否需要多片菊花链：本版只实现单片，多片需另加帧拼接与
   寄存器回读只对 device 0 有效的处理（§10.1.3.3）。
