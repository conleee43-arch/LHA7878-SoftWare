/* ============================================================================
 * LHA7878 驱动 -- 完善版
 * 器件   : Legend Semi LHA7878 (LHA787X 系列 8 通道 24 位同步采样 Delta-Sigma ADC)
 * 依据   : LHA787X Datasheet CN Rev.D (46 页, 2025-10-21)
 *          核对章节: §7.3 推荐工作条件, §7.5 电气特性, §7.6 时间要求 表8,
 *          §7.7 开关特性 表9, §9.3.2 MUX, §9.3.3 模拟输入, §9.3.4 PGA,
 *          §9.3.7 抽取滤波器, §9.3.8 基准, §9.3.9 超范围检测, §9.3.10 GPIO,
 *          §9.4.1 START/建立时间 表15, §9.4.2 RESET, §9.4.3 PWDN,
 *          §9.4.5 数据检索/状态字 图35, §9.5 SPI/命令, §9.5.3.1 多字节命令,
 *          §9.6 寄存器表19/§9.6.1.1~§9.6.1.9, §10.1.2 基本配置,
 *          §10.1.3 多器件, §11.1 上电时序 表39, 图42 上电流程图
 * 平台   : STM32 + CubeMX HAL (SPI2, Mode 1). 使用前在 CubeMX 里确认:
 *          SPI: CPOL=Low, CPHA=2 Edge (即 Mode 1), 8-bit, MSB first,
 *          NSS = Software, SCLK 空闲低; CS/START/RESET/PWDN/DRDY 为 GPIO.
 * 编码   : UTF-8 无 BOM. 本文件不含任何中文 Windows 换行/BOM 依赖.
 *
 * 与原版驱动的主要区别 (原版文件见 workspace/user/files/LHA7878.c/.h):
 *  1. 寄存器表按 Rev.D 重写: 原版 0x0D~0x11 是保留寄存器, 0x15~0x17
 *     在 Rev.D 里根本不存在 (那是另一颗芯片的表), 已删除.
 *  2. 删除 DUT_WTREG / USERKEY UNLOCK / REF_PREQ: Rev.D 里没有这些
 *     寄存器, 原版发出的 0xA1/0x82... 不是合法 WREG 帧, 已删除.
 *     如板子确实需要厂家未公开的解锁流程, 请向 Legend Semi FAE
 *     索要书面说明, 不要把这段写回驱动.
 *  3. 所有 API 带返回值, 初始化会回读校验 ID 与每个写过的寄存器.
 *  4. WREG/RREG 字节间插 tSDECODE (4*tCLK), CS 时序按 §7.6/§9.5.2.1
 *     执行; 读数时 DIN 保持 0x00 (NOP), 不再用 HAL_SPI_Receive
 *     (它在 MOSI 上发 0xFF, 违反 §9.4.5.2 "DIN 应保持低电平").
 *  5. 换算按 §9.5.1 公式 (8): V = code * 2*VREF/Gain / 2^24,
 *     用 int64 计算, 不再写死 5000 mV.
 *  6. 128kSPS + RDATAC 在 Rev.D 下读不完 (216 位需 SCLK >= 36.9MHz,
 *     超过上限 33.3MHz), init() 会直接返回 LHA7878_ERR_RATE,
 *     必须二选一: A) 保持 128kSPS 改 RDATA; B) 降到 64kSPS 保持
 *     RDATAC. 减通道不能缩短帧 (§9.4.5.4 帧恒 216 位).
 * ========================================================================== */
#ifndef LHA7878_H
#define LHA7878_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/* 使用方在 main.h (CubeMX 生成) 里已包含对应型号的 HAL 头文件.
 * 若你的工程不用 main.h, 改为直接包含 stm32f4xx_hal.h 等. */
#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------ 错误码 ---------------------------------- */
typedef enum {
    LHA7878_OK = 0,
    LHA7878_ERR_ARG,        /* 参数非法 (空指针/越界/配置冲突) */
    LHA7878_ERR_HAL,        /* HAL_SPI / HAL_GPIO 返回错误或超时 */
    LHA7878_ERR_TIMEOUT,    /* 等 DRDY / 等建立超时 */
    LHA7878_ERR_ID,         /* ID 校验失败: 不是 LHA7878 */
    LHA7878_ERR_VERIFY,     /* 写寄存器后回读不一致 */
    LHA7878_ERR_RATE,       /* 速率/SCLK 组合在手册下不可行, 见 §9.5.2.2 公式9 */
    LHA7878_ERR_VREF,       /* 基准选择与 AVDD 冲突 (§9.3.8) */
    LHA7878_ERR_FRAME,      /* 状态字同步头不是 1100b (§9.4.5.3 图35) */
    LHA7878_ERR_NOT_READY,  /* 未初始化 / 未进入连续转换 */
    LHA7878_ERR_FAULT,      /* 帧内 FAULT_STATP/N 有置位 */
} lha7878_status_t;

/* ------------------------------ 寄存器 ---------------------------------- */
/* Rev.D §9.6 表19. 0x0D~0x11 为保留寄存器, 只能写默认值 0x00, 不可单独配置;
 * 0x15 及以上不存在. */
#define LHA7878_REG_ID          0x00u
#define LHA7878_REG_CONFIG1     0x01u
#define LHA7878_REG_CONFIG2     0x02u
#define LHA7878_REG_CONFIG3     0x03u
#define LHA7878_REG_FAULT       0x04u
#define LHA7878_REG_CH1SET      0x05u
#define LHA7878_REG_CH8SET      0x0Cu
#define LHA7878_REG_FAULT_STATP 0x12u
#define LHA7878_REG_FAULT_STATN 0x13u
#define LHA7878_REG_GPIO        0x14u
#define LHA7878_REG_COUNT       0x15u

/* ID (§9.6.1.1): REV_ID[7:5]=110b 为 LHA787X, NU_CH[1:0]=00b 为 8 通道.
 * LHA7878 的 ID 期望值 = 0xC0. */
#define LHA7878_ID_EXPECTED     0xC0u
#define LHA7878_ID_REV_MASK     0xE0u
#define LHA7878_ID_REV_VALUE    0xC0u
#define LHA7878_ID_NUCH_MASK    0x03u

/* CONFIG1 (§9.6.1.2, 复位值 0x1E): bit7=0, bit6 DAISY_IN, bit5 CLK_EN,
 * bit4=1, bit3=1, bit2:0 DR. 写 CONFIG1 会触发数字滤波器内部复位. */
#define LHA7878_CONFIG1_FIXED   0x18u   /* bit4/bit3 置 1, 其余 0 */

/* CONFIG2 (§9.6.1.3, 复位值 0xC0): bit7:5 必须写 110b. */
#define LHA7878_CONFIG2_FIXED   0xC0u

/* CONFIG3 (§9.6.1.4, 复位值 0x00) */
#define LHA7878_CONFIG3_PDB_REFBUF  (1u << 7)
#define LHA7878_CONFIG3_VREF_4V     (1u << 5)
#define LHA7878_CONFIG3_OPAMP_REF   (1u << 3)
#define LHA7878_CONFIG3_PDB_OPAMP   (1u << 2)

/* FAULT (§9.6.1.5): COMP_TH[7:5], 高侧 95%->70%, 低侧 5%->30%, 步进 2.5%. */
#define LHA7878_FAULT_COMP_TH_SHIFT 5u

/* CHnSET (§9.6.1.6, 复位值 0x00): bit7 PD, bit6:4 GAIN, bit3=0,
 * bit2:0 MUX. 注意 GAIN 码 000 对应增益 6, 不是 1, 见增益换算表. */
#define LHA7878_CHNSET_PD       (1u << 7)

/* GPIO (§9.6.1.9, 复位值 0x0F): bit7:4 GPIOD, bit3:0 GPIOC (1=输入). */

/* ------------------------------ 命令 ------------------------------------ */
/* §9.5.3 表18. 系统命令/RDATA 在第 7 个 SCLK 下降沿解码,
 * RREG/WREG 在第 8 个下降沿解码. */
#define LHA7878_CMD_WAKEUP      0x02u
#define LHA7878_CMD_STANDBY     0x04u
#define LHA7878_CMD_RESET       0x06u
#define LHA7878_CMD_START       0x08u
#define LHA7878_CMD_STOP        0x0Au
#define LHA7878_CMD_OFFSETCAL   0x1Au
#define LHA7878_CMD_RDATAC      0x10u
#define LHA7878_CMD_SDATAC      0x11u
#define LHA7878_CMD_RDATA       0x12u
/* RREG/WREG 首字节: 001rrrrr / 010rrrrr, rrrrr 为起始寄存器地址. */
#define LHA7878_CMD_RREG_BASE   0x20u
#define LHA7878_CMD_WREG_BASE   0x40u

/* ------------------------------ 数据速率 -------------------------------- */
/* §9.6.1.2 表24 (fCLK=2.048MHz 时). DR 码与速率的对应关系, 不要按数值
 * 大小猜速率. */
typedef enum {
    LHA7878_DR_64K  = 0x00u,
    LHA7878_DR_32K  = 0x01u,
    LHA7878_DR_16K  = 0x02u,
    LHA7878_DR_8K   = 0x03u,
    LHA7878_DR_4K   = 0x04u,
    LHA7878_DR_2K   = 0x05u,
    LHA7878_DR_1K   = 0x06u,
    LHA7878_DR_128K = 0x07u,
} lha7878_dr_t;

/* 速率 (kSPS) 查表, 下标为 DR 码. */
extern const uint16_t lha7878_dr_ksps[8];
/* 建立时间 表15 (单位 tCLK), DR=111 (128kSPS) 手册未给, 此处 80 为
 * 按 64kSPS 的 152 折半的估算值, 仅用于超时上限, 不作验收判据. */
extern const uint16_t lha7878_settle_tclk[8];

/* ------------------------------ 增益 / MUX ------------------------------ */
/* GAIN 码 (§9.6.1.6 表32): 000=6, 001=1, 010=2, 011=3, 100=4,
 * 101=8, 110=12, 111=24. */
typedef enum {
    LHA7878_GAIN_1  = 1,
    LHA7878_GAIN_2  = 2,
    LHA7878_GAIN_3  = 3,
    LHA7878_GAIN_4  = 4,
    LHA7878_GAIN_6  = 6,
    LHA7878_GAIN_8  = 8,
    LHA7878_GAIN_12 = 12,
    LHA7878_GAIN_24 = 24,
} lha7878_gain_t;

/* MUX (§9.6.1.6 表32): 000 正常, 001 短接至 (AVDD+AVSS)/2 (噪声测量),
 * 011 MVDD 电源测量, 100 温度传感器, 101 测试信号; 010/110/111 不用. */
typedef enum {
    LHA7878_MUX_NORMAL = 0u,
    LHA7878_MUX_SHORT  = 1u,
    LHA7878_MUX_MVDD   = 3u,
    LHA7878_MUX_TEMP   = 4u,
    LHA7878_MUX_TEST   = 5u,
} lha7878_mux_t;

/* 基准 (§9.3.8): 默认上电为外部基准模式. 内部 4V 仅 AVDD=5V 时可用,
 * AVDD=3V 时内部基准必须为 2.4V. */
typedef enum {
    LHA7878_VREF_EXT = 0,   /* 外部基准, PDB_REFBUF=0 */
    LHA7878_VREF_INT_2V4,   /* 内部 2.4V, PDB_REFBUF=1, VREF_4V=0 */
    LHA7878_VREF_INT_4V,    /* 内部 4.0V, PDB_REFBUF=1, VREF_4V=1, 仅 AVDD=5V */
} lha7878_vref_t;

/* 读数模式 */
typedef enum {
    LHA7878_MODE_RDATAC = 0,  /* 连续读数, DRDY 后直接移 216 位 */
    LHA7878_MODE_RDATA  = 1,  /* 每帧先发 RDATA 0x12 再移 216 位, 可与下个 DRDY 重叠 */
} lha7878_mode_t;

/* 测试信号 (§9.6.1.3): INT_TEST=1 时内部产生, TEST_FREQ 00=fCLK/2^21,
 * 01=fCLK/2^20, 11=DC; TEST_AMP 两档在 Rev.D 里文字相同, 均为
 * (VREFP-VREFN)/1000, 不要按 1x/2x 理解. */
typedef enum {
    LHA7878_TEST_FREQ_DIV21 = 0u,
    LHA7878_TEST_FREQ_DIV20 = 1u,
    LHA7878_TEST_FREQ_DC    = 3u,
} lha7878_test_freq_t;

/* ------------------------------ 配置结构 -------------------------------- */
typedef struct {
    lha7878_dr_t    data_rate;      /* 默认 8kSPS */
    lha7878_mode_t  mode;           /* 默认 RDATAC; 128kSPS 时必须选 RDATA */
    lha7878_vref_t  vref;           /* 默认 INT_2V4; 板上 AVDD=5V 且要 4V 时改 INT_4V */
    uint32_t        vref_ext_uv;    /* 仅 vref=EXT 时用: 外部基准电压, uV, 如 2400000 */
    uint32_t        avdd_mv;        /* 模拟电源, mV, 如 3000 或 5000, 用于 MVDD 换算与校验 */
    uint32_t        dvdd_mv;        /* 数字电源, mV, 如 1800 或 3300, 用于 CH3/CH4 的 MVDD 换算 */
    uint32_t        clk_hz;         /* 主时钟, 默认 2048000; 外部 1.7~2.25MHz, 内部振荡器 2.048MHz */
    uint32_t        sclk_hz;        /* 读数 SCLK, 默认 10000000; 见 lha7878_min_sclk_hz() 校验 */
    lha7878_gain_t  gain[8];        /* 每通道 PGA 增益, 默认全 1 */
    lha7878_mux_t   mux[8];         /* 每通道 MUX, 默认全 NORMAL */
    bool            channel_pd[8];  /* true=该通道掉电 (输出 0, 帧长不变), 默认全 false */
    uint8_t         fault_th;       /* COMP_TH[2:0], 0=95%/5% ... 7=70%/30%, 默认 0 */
    bool            multi_readback; /* true=CONFIG1 DAISY_IN=1 (单片推荐, 允许多次回读);
                                     * false=0 (菊花链模式, 本驱动不实现多片菊花链) */
    bool            clk_en;         /* CLK_EN, 仅 CLKSEL=1 且要把内部时钟输出到 CLK 脚时置 1 */
    bool            test_int;       /* CONFIG2 INT_TEST */
    uint8_t         test_freq;      /* TEST_FREQ[1:0], 见 lha7878_test_freq_t */
    uint8_t         gpio_dir;       /* GPIOC[4:1], 1=输入 (复位值 0x0F), 0=输出 */
    uint8_t         gpio_out;       /* GPIOD[4:1] 输出值 (仅输出方向有效) */
} lha7878_config_t;

typedef struct {
    SPI_HandleTypeDef *hspi;
    GPIO_TypeDef *cs_port;    uint16_t cs_pin;
    GPIO_TypeDef *start_port; uint16_t start_pin;
    GPIO_TypeDef *reset_port; uint16_t reset_pin;
    GPIO_TypeDef *pwdn_port;  uint16_t pwdn_pin;
    GPIO_TypeDef *drdy_port;  uint16_t drdy_pin;  /* DRDY 低有效 */
    lha7878_config_t cfg;
    /* 运行时状态 (由驱动维护, 使用方只读) */
    bool     initialized;
    bool     in_rdatac;
    uint32_t frame_count;
    uint32_t frame_error_count;
    uint32_t skipped_frames;  /* RDATA 模式下取不完时, 由使用方在超时后自增, 驱动不猜 */
} lha7878_dev_t;

/* 一帧数据: 27 字节 = 24 位状态字 + 8 通道 x 24 位 (§9.4.5.4, 恒 216 位) */
#define LHA7878_FRAME_BYTES 27u
#define LHA7878_FRAME_BITS  216u

typedef struct {
    uint32_t status_raw;   /* 24 位状态字: [23:20]=1100b, [19:12]=FAULT_STATP,
                            * [11:4]=FAULT_STATN, [3:0]=GPIO(GPIOD[4:1]) */
    uint8_t  fault_p;      /* FAULT_STATP, 每位对应一通道正输入超阈值 */
    uint8_t  fault_n;      /* FAULT_STATN, 每位对应一通道负输入超阈值 */
    uint8_t  gpio;         /* 状态字里的 GPIO 值 */
    int32_t  raw[8];       /* 24 位补码已符号扩展到 32 位, 满量程 ±8388607 */
    uint32_t frame_index;  /* 驱动累计的帧序号 */
} lha7878_frame_t;

/* ------------------------------ 默认配置 -------------------------------- */
/* 填一个安全的默认配置: 8kSPS / RDATAC / 内部 2.4V / 增益 1 / 正常输入.
 * AVDD/DVDD/CLK/SCLK 必须按实际板子改, 否则 init() 会报 ERR_ARG/ERR_VREF. */
void lha7878_config_default(lha7878_config_t *cfg);

/* 按原工程的 CubeMX 引脚名填 dev (hspi2, GPIOB 的 PWDN/RESET/START,
 * SPI2 CS, DRDY). 若你的 CubeMX 引脚名不同, 不要改驱动, 改这里的
 * 赋值或直接手动填 lha7878_dev_t 的各个字段. */
void lha7878_dev_default(lha7878_dev_t *dev, const lha7878_config_t *cfg);

/* ------------------------------ 基础 API -------------------------------- */
/* 完整初始化: 上电时序 (§11.1/图42) -> 硬件复位 -> SDATAC -> 读 ID 校验 ->
 * 写 CONFIG1/2/3/FAULT/CHnSET 并回读校验 -> (可选) OFFSETCAL ->
 * START 拉高 -> 进入配置里的读数模式. 失败时返回具体错误码, 不静默. */
lha7878_status_t lha7878_init(lha7878_dev_t *dev);

/* 仅做上电+复位+SDATAC+ID 校验, 不写配置寄存器. 调试/分步上板时用. */
lha7878_status_t lha7878_powerup(lha7878_dev_t *dev);
lha7878_status_t lha7878_reset_hw(lha7878_dev_t *dev);
lha7878_status_t lha7878_reset_cmd(lha7878_dev_t *dev);

/* 读 ID 并校验 (期望 0xC0). */
lha7878_status_t lha7878_read_id(lha7878_dev_t *dev, uint8_t *id);

/* 单寄存器读写 (WREG/RREG, 字节间插 tSDECODE, 写后不自动回读;
 * 需要校验时用 lha7878_write_reg_verify). */
lha7878_status_t lha7878_read_reg(lha7878_dev_t *dev, uint8_t addr, uint8_t *value);
lha7878_status_t lha7878_write_reg(lha7878_dev_t *dev, uint8_t addr, uint8_t value);
lha7878_status_t lha7878_write_reg_verify(lha7878_dev_t *dev, uint8_t addr, uint8_t value);
/* 连续读写: addr 为起始地址, count 为寄存器数. 跨保留寄存器 0x0D~0x11 时
 * 调用方必须在 data 里按手册默认值 0x00 填好, 驱动不替你猜. */
lha7878_status_t lha7878_read_regs(lha7878_dev_t *dev, uint8_t addr, uint8_t *data, uint8_t count);
lha7878_status_t lha7878_write_regs(lha7878_dev_t *dev, uint8_t addr, const uint8_t *data, uint8_t count);

/* 写全部通道设置 CH1SET~CH8SET (0x05~0x0C, 一次 WREG, 8 字节). */
lha7878_status_t lha7878_write_all_channels(lha7878_dev_t *dev);
/* 改单个通道的增益/MUX/掉电位并写回该通道寄存器. */
lha7878_status_t lha7878_set_channel(lha7878_dev_t *dev, uint8_t ch,
                                     lha7878_gain_t gain, lha7878_mux_t mux, bool pd);

/* ------------------------------ 命令 ------------------------------------ */
/* 单字节命令 (CS 时序 + 命令后等待按 §9.5.3 处理, RESET 固定等 2ms). */
lha7878_status_t lha7878_cmd(lha7878_dev_t *dev, uint8_t cmd);
lha7878_status_t lha7878_wakeup(lha7878_dev_t *dev);
lha7878_status_t lha7878_standby(lha7878_dev_t *dev);
lha7878_status_t lha7878_start(lha7878_dev_t *dev);   /* START 命令, START 脚保持低 */
lha7878_status_t lha7878_stop(lha7878_dev_t *dev);
lha7878_status_t lha7878_start_pin(lha7878_dev_t *dev, bool high); /* START 脚控制 */
lha7878_status_t lha7878_offset_cal(lha7878_dev_t *dev); /* 约 153ms, 见 §9.5.3.7 */

/* 校准诊断记录: [0]=校准后硬件自改的CONFIG1(预期0x5E), [1]=目标恢复值(0x5B), [2]=校验状态(0=OK), [3]=验证读回值(0x5B) */
extern volatile uint8_t g_cal_log[8];

/* ------------------------------ 读数 ------------------------------------ */
lha7878_status_t lha7878_enter_rdatac(lha7878_dev_t *dev);
lha7878_status_t lha7878_exit_rdatac(lha7878_dev_t *dev); /* 即 SDATAC */
lha7878_status_t lha7878_wait_drdy(lha7878_dev_t *dev, uint32_t timeout_ms);
bool             lha7878_drdy_is_low(const lha7878_dev_t *dev);

/* RDATAC 模式: 等 DRDY 低 -> CS 低 -> 移 27 字节 (DIN=0x00) -> 解析.
 * 超时返回 ERR_TIMEOUT, 状态字同步头不对返回 ERR_FRAME. */
lha7878_status_t lha7878_read_frame_rdatac(lha7878_dev_t *dev, lha7878_frame_t *frame,
                                           uint32_t timeout_ms);
/* RDATA 模式: 发 RDATA 0x12 (同一 CS 周期内) -> 紧接着移 27 字节 -> 解析.
 * 128kSPS 下 20MHz 约需 11.2us, 会跨一个 DRDY, 手册允许重叠 (§9.5.3.10),
 * 但取不完每一帧时会静默跳帧, 使用方须用 frame_index / 状态字核对连续性. */
lha7878_status_t lha7878_read_frame_rdata(lha7878_dev_t *dev, lha7878_frame_t *frame,
                                          uint32_t timeout_ms);
/* 按当前配置的 mode 自动选上面两个函数. */
lha7878_status_t lha7878_read_frame(lha7878_dev_t *dev, lha7878_frame_t *frame,
                                    uint32_t timeout_ms);

/* 纯解析: buf 为 27 字节原始帧, 不碰硬件. 单元测试/离线回放时用. */
lha7878_status_t lha7878_parse_frame(const uint8_t *buf, lha7878_frame_t *frame,
                                     uint32_t frame_index);

/* ------------------------------ 换算 ------------------------------------ */
/* §9.5.1 公式 (8): 1 LSB = 2*VREF/Gain / 2^24. 用 int64 计算, 返回 uV. */
int64_t  lha7878_code_to_uv(int32_t code, uint32_t vref_uv, lha7878_gain_t gain);
int32_t  lha7878_code_to_mv(int32_t code, uint32_t vref_uv, lha7878_gain_t gain);
float    lha7878_code_to_volt(int32_t code, uint32_t vref_uv, lha7878_gain_t gain);
/* 按 dev 当前配置把一帧的 8 通道换算成 uV (调用方提供 int64_t uv[8]). */
void     lha7878_frame_to_uv(const lha7878_dev_t *dev, const lha7878_frame_t *frame, int64_t *uv);
/* 当前配置下的 VREF (uV). 外部基准时返回 cfg 里约定的值, 见下. */
uint32_t lha7878_vref_uv(const lha7878_dev_t *dev);

/* 温度 (§9.3.2.3 公式 3, 增益须为 1): T = (Vuv-145300)/490 + 25.
 * 返回值单位 0.01 摄氏度 (如 2534 = 25.34 摄氏度), 避免浮点. */
int32_t  lha7878_temp_centi_c(int32_t code, uint32_t vref_uv);
/* 电源测量 (§9.3.2.4, 增益须为 1): CH1/2/5/6/7/8 量 0.5*(AVDD-AVSS),
 * CH3/4 量 DVDD/4. 返回被测电源的 mV (AVSS=0 假设). */
int32_t  lha7878_mvdd_mv(int32_t code, uint32_t vref_uv, uint8_t ch,
                         uint32_t avdd_mv, uint32_t dvdd_mv);

/* ------------------------------ 故障 / GPIO / 自检 ----------------------- */
lha7878_status_t lha7878_read_fault_status(lha7878_dev_t *dev, uint8_t *fault_p, uint8_t *fault_n);
lha7878_status_t lha7878_set_fault_threshold(lha7878_dev_t *dev, uint8_t comp_th);
lha7878_status_t lha7878_gpio_read(lha7878_dev_t *dev, uint8_t *gpiod);
lha7878_status_t lha7878_gpio_write(lha7878_dev_t *dev, uint8_t gpiod_out, uint8_t gpioc_dir);
/* 自检 (图42 的后半段, 不在 init 里自动做): 先 SDATAC, 按入参改 MUX/
 * CONFIG2, 读一帧, 再恢复原配置并回到 RDATAC/RDATA. 调用方负责在
 * 转换稳定后 (至少 3 个 DRDY, §9.4.1.2) 再判断读数, 不要拿第一帧当结论. */
lha7878_status_t lha7878_selftest_short(lha7878_dev_t *dev, lha7878_frame_t *frame);
lha7878_status_t lha7878_selftest_signal(lha7878_dev_t *dev, lha7878_frame_t *frame);

/* ------------------------------ 时序校验 -------------------------------- */
/* RDATAC 下读完一帧所需的最低 SCLK (Hz), §9.5.2.2 公式 9:
 * SCLK_min = 216 / (tDR - 4*tCLK). 128kSPS 时约 36.9MHz. */
uint32_t lha7878_min_sclk_hz(lha7878_dr_t dr, uint32_t clk_hz);
/* 校验当前配置 + SCLK 是否可行: RDATAC 且 min > 33.3MHz 时返回 ERR_RATE;
 * RDATA 模式不做此校验 (手册未给 RDATA 的硬上限, 见头文件说明). */
lha7878_status_t lha7878_validate_timing(const lha7878_dev_t *dev);
/* tSDECODE = 4*tCLK (us, 向上取整, 至少 2us). */
uint32_t lha7878_tsdecode_us(const lha7878_dev_t *dev);

/* ------------------------------ 延时 (可替换) --------------------------- */
/* 默认实现: DWT CYCCNT (已在 init 里使能), 精度 1us; 若 DWT 不可用则
 * 退化为按 SystemCoreClock 估算的 NOP 循环 (已在注释里标为估算).
 * 需要更高精度/低功耗时, 使用方可在自己的文件里重定义这两个弱函数. */
void lha7878_delay_us(uint32_t us);
void lha7878_delay_ms(uint32_t ms);

#ifdef __cplusplus
}
#endif
#endif /* LHA7878_H */
