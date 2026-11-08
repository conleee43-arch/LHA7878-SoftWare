/* LHA7878 驱动 -- 完善版, 依据 LHA787X Datasheet CN Rev.D.
 * 编码 UTF-8 无 BOM. 详见 LHA7878.h 头部说明与核对章节清单. */
#include "LHA7878.h"
#include <stdio.h>

#ifndef __weak
#define __weak __attribute__((weak))
#endif

/* ------------------------------ 查表 ------------------------------------ */
const uint16_t lha7878_dr_ksps[8] = { 64u, 32u, 16u, 8u, 4u, 2u, 1u, 128u };
/* 表15 建立时间 (tCLK). DR=111 的 80 为估算值, 见头文件. */
const uint16_t lha7878_settle_tclk[8] = { 152u, 296u, 584u, 1160u, 2312u, 4616u, 9224u, 80u };

/* 增益 <-> CHnSET 码表 (§9.6.1.6 表32). 码 000 对应增益 6, 务必查表, 不要移位推算. */
static const struct { lha7878_gain_t gain; uint8_t code; } s_gain_table[] = {
    { LHA7878_GAIN_6, 0u }, { LHA7878_GAIN_1, 1u }, { LHA7878_GAIN_2, 2u },
    { LHA7878_GAIN_3, 3u }, { LHA7878_GAIN_4, 4u }, { LHA7878_GAIN_8, 5u },
    { LHA7878_GAIN_12, 6u }, { LHA7878_GAIN_24, 7u },
};

static bool gain_to_code(lha7878_gain_t gain, uint8_t *code)
{
    for (size_t i = 0; i < sizeof(s_gain_table) / sizeof(s_gain_table[0]); i++) {
        if (s_gain_table[i].gain == gain) { *code = s_gain_table[i].code; return true; }
    }
    return false;
}

static bool mux_is_valid(uint8_t mux)
{
    return (mux == LHA7878_MUX_NORMAL) || (mux == LHA7878_MUX_SHORT) ||
           (mux == LHA7878_MUX_MVDD) || (mux == LHA7878_MUX_TEMP) ||
           (mux == LHA7878_MUX_TEST);
}

static uint8_t chnset_byte(lha7878_gain_t gain, lha7878_mux_t mux, bool pd)
{
    uint8_t code = 0;
    (void)gain_to_code(gain, &code);
    return (uint8_t)((pd ? LHA7878_CHNSET_PD : 0u) | (uint8_t)(code << 4) |
                     (uint8_t)mux);
}

/* ------------------------------ 延时 ------------------------------------ */
/* DWT CYCCNT 精确延时. __weak: 使用方可在自己文件里重定义这两个函数. */
__weak void lha7878_delay_us(uint32_t us)
{
    if (us == 0u) { return; }
    /* DWT 已在 lha7878_init() 里使能; 若未使能, 走下面的估算分支. */
    if ((CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk) != 0u &&
        (DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) != 0u && (SystemCoreClock != 0u)) {
        uint32_t start = DWT->CYCCNT;
        uint32_t ticks = (uint32_t)((uint64_t)us * (uint64_t)(SystemCoreClock / 1000000u));
        while ((DWT->CYCCNT - start) < ticks) { __NOP(); }
        return;
    }
    /* 估算分支: 每轮循环约 4 个 CPU 周期, 仅作 tSDECODE 这类 us 级等待,
     * 不要把它当作 ms 级精确定时用. */
    uint32_t loops = (SystemCoreClock / 1000000u) * us / 4u;
    for (volatile uint32_t i = 0; i < loops; i++) { __NOP(); }
}

__weak void lha7878_delay_ms(uint32_t ms)
{
    HAL_Delay(ms);
}

static void delay_4tclk(const lha7878_dev_t *dev)
{
    /* tSDECODE = 4*tCLK, 也是 tSCCS / 命令后等待的下限 (§7.6 表8). */
    lha7878_delay_us(lha7878_tsdecode_us(dev));
}

static void delay_2tclk(const lha7878_dev_t *dev)
{
    uint32_t us = lha7878_tsdecode_us(dev) / 2u;
    lha7878_delay_us((us == 0u) ? 1u : us);
}

uint32_t lha7878_tsdecode_us(const lha7878_dev_t *dev)
{
    uint32_t clk = (dev == NULL) ? 2048000u : dev->cfg.clk_hz;
    if (clk == 0u) { clk = 2048000u; }
    /* 4*tCLK, 单位 us, 向上取整, 至少 2us (CLK=2.048MHz 时为 1.96us -> 2us). */
    uint32_t us = (uint32_t)((4000000u + clk - 1u) / clk);
    return (us < 2u) ? 2u : us;
}

/* ------------------------------ CS / SPI 底层 ---------------------------- */
static void cs_assert(const lha7878_dev_t *dev)
{
    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_RESET);
    /* tCSSC: CS 下降沿到第一个 SCLK 上升沿 >= 6ns/17ns, 1us 已覆盖. */
    lha7878_delay_us(1u);
}

static void cs_release(const lha7878_dev_t *dev)
{
    /* tSCCS: 通信结束后等 >= 4tCLK 再拉高 CS (§9.5.2.1). */
    delay_4tclk(dev);
    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_SET);
    /* tCSH: CS 高电平至少 2tCLK, 下次拉低前已满足. */
    delay_2tclk(dev);
}

/* 单字节全双工收发. tx 为要发出的字节, 返回收到的字节. */
static lha7878_status_t spi_xfer_byte(lha7878_dev_t *dev, uint8_t tx, uint8_t *rx)
{
    uint8_t r = 0;
    HAL_StatusTypeDef st = HAL_SPI_TransmitReceive(dev->hspi, &tx, &r, 1u, 100u);
    if (st != HAL_OK) { return LHA7878_ERR_HAL; }
    if (rx != NULL) { *rx = r; }
    return LHA7878_OK;
}

/* 多字节收发: tx 全 0x00 时即为纯读数 (DIN 保持低, §9.4.5.2).
 * 注意: 不要用 HAL_SPI_Receive 读数, 它在 MOSI 上发 0xFF. */
static lha7878_status_t spi_xfer_buf(lha7878_dev_t *dev, const uint8_t *tx,
                                     uint8_t *rx, uint16_t len)
{
    if (len == 0u) { return LHA7878_OK; }
    HAL_StatusTypeDef st = HAL_SPI_TransmitReceive(dev->hspi,
                                                   (uint8_t *)tx, rx, len, 1000u);
    return (st == HAL_OK) ? LHA7878_OK : LHA7878_ERR_HAL;
}

/* ------------------------------ 配置校验 -------------------------------- */
uint32_t lha7878_vref_uv(const lha7878_dev_t *dev)
{
    if (dev == NULL) { return 2400000u; }
    switch (dev->cfg.vref) {
        case LHA7878_VREF_INT_2V4: return 2400000u;
        case LHA7878_VREF_INT_4V:  return 4000000u;
        case LHA7878_VREF_EXT:
        default: return dev->cfg.vref_ext_uv;
    }
}

uint32_t lha7878_min_sclk_hz(lha7878_dr_t dr, uint32_t clk_hz)
{
    if ((uint8_t)dr > 7u || clk_hz == 0u) { return 0u; }
    /* tDR = 1/速率, 分母 = tDR - 4*tCLK, 分子 216 位 (§9.5.2.2 公式9). */
    uint64_t rate_hz = (uint64_t)lha7878_dr_ksps[(uint8_t)dr] * 1000u;
    uint64_t tdr_ns = 1000000000ull / rate_hz;
    uint64_t tclk_ns = 1000000000ull / (uint64_t)clk_hz;
    uint64_t denom_ns = (tdr_ns > 4u * tclk_ns) ? (tdr_ns - 4u * tclk_ns) : 0u;
    if (denom_ns == 0u) { return 0xFFFFFFFFu; }
    return (uint32_t)((216ull * 1000000000ull) / denom_ns);
}

static uint32_t sclk_ceiling_hz(const lha7878_dev_t *dev)
{
    /* §7.6 表8 脉宽限制: DVDD 2.7~3.6V 时 15+15ns -> 33.3MHz,
     * DVDD 1.7~2.0V 时 25+25ns -> 20MHz. 实操建议再留裕量, 见头文件. */
    return (dev->cfg.dvdd_mv < 2000u) ? 20000000u : 33300000u;
}

lha7878_status_t lha7878_validate_timing(const lha7878_dev_t *dev)
{
    if (dev == NULL) { return LHA7878_ERR_ARG; }
    if (dev->cfg.sclk_hz > sclk_ceiling_hz(dev)) { return LHA7878_ERR_RATE; }
    if (dev->cfg.mode == LHA7878_MODE_RDATAC) {
        uint32_t min_hz = lha7878_min_sclk_hz(dev->cfg.data_rate, dev->cfg.clk_hz);
        if (dev->cfg.sclk_hz < min_hz) { return LHA7878_ERR_RATE; }
    }
    /* RDATA 模式: 手册只说读可与下个 DRDY 重叠, 未给硬上限, 不在此校验. */
    return LHA7878_OK;
}

static lha7878_status_t validate_config(const lha7878_dev_t *dev)
{
    if (dev == NULL || dev->hspi == NULL || dev->cs_port == NULL ||
        dev->start_port == NULL || dev->reset_port == NULL ||
        dev->pwdn_port == NULL || dev->drdy_port == NULL) {
        return LHA7878_ERR_ARG;
    }
    const lha7878_config_t *c = &dev->cfg;
    if ((uint8_t)c->data_rate > 7u || c->clk_hz < 1000000u || c->clk_hz > 2250000u ||
        c->avdd_mv < 2700u || c->avdd_mv > 5250u ||
        c->dvdd_mv < 1700u || c->dvdd_mv > 3600u || c->sclk_hz == 0u ||
        c->fault_th > 7u) {
        return LHA7878_ERR_ARG;
    }
    if (c->vref == LHA7878_VREF_INT_4V && c->avdd_mv < 4500u) {
        return LHA7878_ERR_VREF;  /* §9.3.8: 4V 基准仅 AVDD=5V 时可用 */
    }
    if (c->vref == LHA7878_VREF_EXT &&
        (c->vref_ext_uv < 1000000u || c->vref_ext_uv > 4000000u ||
         c->vref_ext_uv > c->avdd_mv * 1000u)) {
        return LHA7878_ERR_VREF;
    }
    for (uint8_t i = 0; i < 8u; i++) {
        uint8_t code = 0;
        if (!gain_to_code(c->gain[i], &code) || !mux_is_valid((uint8_t)c->mux[i])) {
            return LHA7878_ERR_ARG;
        }
    }
    return lha7878_validate_timing(dev);
}

void lha7878_config_default(lha7878_config_t *cfg)
{
    if (cfg == NULL) { return; }
    cfg->data_rate = LHA7878_DR_8K;
    cfg->mode = LHA7878_MODE_RDATAC;
    cfg->vref = LHA7878_VREF_INT_2V4;
    cfg->vref_ext_uv = 2400000u;
    cfg->avdd_mv = 3000u;
    cfg->dvdd_mv = 3300u;
    cfg->clk_hz = 2048000u;
    cfg->sclk_hz = 10000000u;
    for (uint8_t i = 0; i < 8u; i++) {
        cfg->gain[i] = LHA7878_GAIN_1;
        cfg->mux[i] = LHA7878_MUX_NORMAL;
        cfg->channel_pd[i] = false;
    }
    cfg->fault_th = 0u;
    cfg->multi_readback = true;
    cfg->clk_en = false;
    cfg->test_int = false;
    cfg->test_freq = (uint8_t)LHA7878_TEST_FREQ_DIV21;
    cfg->gpio_dir = 0x0Fu;
    cfg->gpio_out = 0x00u;
}

void lha7878_dev_default(lha7878_dev_t *dev, const lha7878_config_t *cfg)
{
    if (dev == NULL) { return; }
    /* 原工程的 CubeMX 引脚名. DRDY 在原版轮询代码里是 GPIOB bit0 (PB0),
     * 若 CubeMX 里不是 PB0, 改这一行, 不要改驱动其他地方. */
    extern SPI_HandleTypeDef hspi2;
    dev->hspi = &hspi2;
    dev->cs_port = LHA7878_SPI2_CS_GPIO_Port;
    dev->cs_pin = LHA7878_SPI2_CS_Pin;
    dev->start_port = GPIOB;
    dev->start_pin = LHA7878_START_Pin;
    dev->reset_port = GPIOB;
    dev->reset_pin = LHA7878_nRESET_Pin;
    dev->pwdn_port = GPIOB;
    dev->pwdn_pin = LHA7878_nPWDN_Pin;
    dev->drdy_port = GPIOB;
    dev->drdy_pin = GPIO_PIN_0;
    if (cfg != NULL) { dev->cfg = *cfg; } else { lha7878_config_default(&dev->cfg); }
    dev->initialized = false;
    dev->in_rdatac = false;
    dev->frame_count = 0u;
    dev->frame_error_count = 0u;
    dev->skipped_frames = 0u;
}

/* ------------------------------ 上电 / 复位 ------------------------------ */
lha7878_status_t lha7878_reset_hw(lha7878_dev_t *dev)
{
    if (dev == NULL) { return LHA7878_ERR_ARG; }
    /* §9.4.2: RESET 引脚拉低, 最小脉冲 2*tCLK; 释放后等 2ms (RESET 命令
     * 的执行时间, §9.5.3.4), 再等 18*tCLK 让寄存器恢复默认值. */
    HAL_GPIO_WritePin(dev->reset_port, dev->reset_pin, GPIO_PIN_RESET);
    lha7878_delay_ms(1u);
    HAL_GPIO_WritePin(dev->reset_port, dev->reset_pin, GPIO_PIN_SET);
    lha7878_delay_ms(2u);
    delay_4tclk(dev);
    dev->in_rdatac = true;  /* 复位后默认 RDATAC 模式 (§9.5.3.8) */
    return LHA7878_OK;
}

lha7878_status_t lha7878_powerup(lha7878_dev_t *dev)
{
    lha7878_status_t st;
    if ((st = validate_config(dev)) != LHA7878_OK) { return st; }

    /* DWT 使能, 供 lha7878_delay_us() 用. */
    if ((CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk) == 0u) {
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    }
    DWT->CYCCNT = 0u;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    /* §11.1/图42: 上电前所有数字/模拟输入须为低; PWDN/RESET 置高后
     * 等 100ms tPOR (表39 推荐值), 再等 VCAP1>1.1V.
     * VCAP1 无 MCU 可读引脚时, 以 100ms 等待代替, 并在注释里说明. */
    HAL_GPIO_WritePin(dev->start_port, dev->start_pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(dev->pwdn_port, dev->pwdn_pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(dev->reset_port, dev->reset_pin, GPIO_PIN_SET);
    lha7878_delay_ms(100u);

    st = lha7878_reset_hw(dev);
    if (st != LHA7878_OK) { return st; }

    /* 复位后默认 RDATAC, 寄存器不可写, 必须先 SDATAC (§9.5.3.8/图42). */
    st = lha7878_exit_rdatac(dev);
    if (st != LHA7878_OK) { return st; }

    uint8_t id = 0;
    return lha7878_read_id(dev, &id);
}

lha7878_status_t lha7878_read_id(lha7878_dev_t *dev, uint8_t *id)
{
    lha7878_status_t st;
    uint8_t v = 0;
    if (dev == NULL || id == NULL) { return LHA7878_ERR_ARG; }
    st = lha7878_read_reg(dev, LHA7878_REG_ID, &v);
    if (st != LHA7878_OK) { return st; }
    /* §9.6.1.1: REV_ID=110b, 保留位 4:2 读 0, NU_CH=00b 为 8 通道. */
    if ((v & LHA7878_ID_REV_MASK) != LHA7878_ID_REV_VALUE ||
        (v & 0x1Cu) != 0u || (v & LHA7878_ID_NUCH_MASK) != 0u) {
        return LHA7878_ERR_ID;
    }
    *id = v;
    return LHA7878_OK;
}

/* ------------------------------ 寄存器读写 ------------------------------ */
/* RREG: 首字节 001rrrrr, 次字节 000nnnnn (n=寄存器数-1), 随后每寄存器
 * 1 字节数据, 第 17 个 SCLK 上升沿给出首个寄存器 MSB (§9.5.3.11 图40).
 * 字节间一律插 tSDECODE, CS 全程保持低. */
lha7878_status_t lha7878_read_regs(lha7878_dev_t *dev, uint8_t addr, uint8_t *data, uint8_t count)
{
    lha7878_status_t st;
    if (dev == NULL || data == NULL || count == 0u || count > 16u ||
        addr >= LHA7878_REG_COUNT || (uint16_t)addr + count > LHA7878_REG_COUNT) {
        return LHA7878_ERR_ARG;
    }
    if (dev->in_rdatac) { return LHA7878_ERR_ARG; }  /* RDATAC 下 RREG 被忽略 (§9.5.3 表18 注1) */
    cs_assert(dev);
    st = spi_xfer_byte(dev, (uint8_t)(LHA7878_CMD_RREG_BASE | addr), NULL);
    if (st == LHA7878_OK) { delay_4tclk(dev); }
    if (st == LHA7878_OK) {
        st = spi_xfer_byte(dev, (uint8_t)(count - 1u), NULL);
    }
    if (st == LHA7878_OK) { delay_4tclk(dev); }
    for (uint8_t i = 0; i < count && st == LHA7878_OK; i++) {
        st = spi_xfer_byte(dev, 0x00u, &data[i]);  /* 读数时 DIN 发 0x00 */
        if (i + 1u < count) { delay_4tclk(dev); }
    }
    cs_release(dev);
    return st;
}

lha7878_status_t lha7878_read_reg(lha7878_dev_t *dev, uint8_t addr, uint8_t *value)
{
    return lha7878_read_regs(dev, addr, value, 1u);
}

/* WREG: 首字节 010rrrrr, 次字节 000nnnnn, 随后数据 (MSB first).
 * 跨保留寄存器 0x0D~0x11 时, 调用方须在 data 里按默认值 0x00 填好. */
lha7878_status_t lha7878_write_regs(lha7878_dev_t *dev, uint8_t addr, const uint8_t *data, uint8_t count)
{
    lha7878_status_t st;
    if (dev == NULL || data == NULL || count == 0u || count > 16u ||
        addr >= LHA7878_REG_COUNT || (uint16_t)addr + count > LHA7878_REG_COUNT) {
        return LHA7878_ERR_ARG;
    }
    if (dev->in_rdatac) { return LHA7878_ERR_ARG; }
    cs_assert(dev);
    st = spi_xfer_byte(dev, (uint8_t)(LHA7878_CMD_WREG_BASE | addr), NULL);
    if (st == LHA7878_OK) { delay_4tclk(dev); }
    if (st == LHA7878_OK) {
        st = spi_xfer_byte(dev, (uint8_t)(count - 1u), NULL);
    }
    if (st == LHA7878_OK) { delay_4tclk(dev); }
    for (uint8_t i = 0; i < count && st == LHA7878_OK; i++) {
        st = spi_xfer_byte(dev, data[i], NULL);
        if (i + 1u < count) { delay_4tclk(dev); }
    }
    cs_release(dev);
    /* 写 CONFIG1 会触发滤波器内部复位 (§9.4.2), 调用方随后等 DRDY 再读数. */
    return st;
}

lha7878_status_t lha7878_write_reg(lha7878_dev_t *dev, uint8_t addr, uint8_t value)
{
    return lha7878_write_regs(dev, addr, &value, 1u);
}

lha7878_status_t lha7878_write_reg_verify(lha7878_dev_t *dev, uint8_t addr, uint8_t value)
{
    lha7878_status_t st = lha7878_write_reg(dev, addr, value);
    uint8_t back = 0;
    if (st != LHA7878_OK) { return st; }
    st = lha7878_read_reg(dev, addr, &back);
    if (st != LHA7878_OK) { return st; }
    return (back == value) ? LHA7878_OK : LHA7878_ERR_VERIFY;
}

lha7878_status_t lha7878_write_all_channels(lha7878_dev_t *dev)
{
    uint8_t buf[8];
    if (dev == NULL) { return LHA7878_ERR_ARG; }
    for (uint8_t i = 0; i < 8u; i++) {
        buf[i] = chnset_byte(dev->cfg.gain[i], dev->cfg.mux[i], dev->cfg.channel_pd[i]);
    }
    return lha7878_write_regs(dev, LHA7878_REG_CH1SET, buf, 8u);
}

lha7878_status_t lha7878_set_channel(lha7878_dev_t *dev, uint8_t ch,
                                     lha7878_gain_t gain, lha7878_mux_t mux, bool pd)
{
    uint8_t code = 0;
    if (dev == NULL || ch >= 8u || !gain_to_code(gain, &code) ||
        !mux_is_valid((uint8_t)mux)) {
        return LHA7878_ERR_ARG;
    }
    dev->cfg.gain[ch] = gain;
    dev->cfg.mux[ch] = mux;
    dev->cfg.channel_pd[ch] = pd;
    return lha7878_write_reg_verify(dev, (uint8_t)(LHA7878_REG_CH1SET + ch),
                                    chnset_byte(gain, mux, pd));
}

/* ------------------------------ 单字节命令 ------------------------------ */
lha7878_status_t lha7878_cmd(lha7878_dev_t *dev, uint8_t cmd)
{
    lha7878_status_t st;
    if (dev == NULL) { return LHA7878_ERR_ARG; }
    cs_assert(dev);
    st = spi_xfer_byte(dev, cmd, NULL);
    cs_release(dev);  /* cs_release 里已等 4tCLK, 满足 tSCCS */
    if (st != LHA7878_OK) { return st; }
    switch (cmd) {
        case LHA7878_CMD_RESET:
            /* §9.5.3.4: 执行复位需 2ms, 期间不要发任何命令. */
            lha7878_delay_ms(2u);
            dev->in_rdatac = true;
            break;
        case LHA7878_CMD_RDATAC:
            dev->in_rdatac = true;
            delay_4tclk(dev);  /* 发完 RDATAC 后取数/SDATAC 前等 4tCLK */
            break;
        case LHA7878_CMD_SDATAC:
            dev->in_rdatac = false;
            delay_4tclk(dev);  /* 下个命令前等 4tCLK (§9.5.3.9) */
            break;
        case LHA7878_CMD_WAKEUP:
        case LHA7878_CMD_START:
        case LHA7878_CMD_STOP:
        default:
            delay_4tclk(dev);
            break;
    }
    return LHA7878_OK;
}

lha7878_status_t lha7878_reset_cmd(lha7878_dev_t *dev)
{
    return lha7878_cmd(dev, LHA7878_CMD_RESET);
}

lha7878_status_t lha7878_wakeup(lha7878_dev_t *dev)
{
    lha7878_status_t st = lha7878_cmd(dev, LHA7878_CMD_WAKEUP);
    if (st != LHA7878_OK) { return st; }
    /* 待机唤醒时间 31.25us (§7.5), 留 1ms 裕量. 上电唤醒 (150ms) 不走这里. */
    lha7878_delay_ms(1u);
    return LHA7878_OK;
}

lha7878_status_t lha7878_standby(lha7878_dev_t *dev)
{
    return lha7878_cmd(dev, LHA7878_CMD_STANDBY);
}

lha7878_status_t lha7878_start(lha7878_dev_t *dev)
{
    return lha7878_cmd(dev, LHA7878_CMD_START);
}

lha7878_status_t lha7878_stop(lha7878_dev_t *dev)
{
    return lha7878_cmd(dev, LHA7878_CMD_STOP);
}

lha7878_status_t lha7878_start_pin(lha7878_dev_t *dev, bool high)
{
    if (dev == NULL) { return LHA7878_ERR_ARG; }
    /* §9.4.1: START 脚拉高至少 2*tCLK 才开始转换. */
    HAL_GPIO_WritePin(dev->start_port, dev->start_pin,
                      high ? GPIO_PIN_SET : GPIO_PIN_RESET);
    if (high) { delay_2tclk(dev); }
    return LHA7878_OK;
}

lha7878_status_t lha7878_offset_cal(lha7878_dev_t *dev)
{
    lha7878_status_t st;
    if (dev == NULL) { return LHA7878_ERR_ARG; }
    /* §9.5.3.7: 每次改 PGA 增益后建议发一次. 总时间
     * (19*TDR*8)+1ms = 153ms (1kSPS 下), 等 200ms 再返回. */
    st = lha7878_cmd(dev, LHA7878_CMD_OFFSETCAL);
    if (st != LHA7878_OK) { return st; }
    lha7878_delay_ms(200u);
    return LHA7878_OK;
}

lha7878_status_t lha7878_enter_rdatac(lha7878_dev_t *dev)
{
    return lha7878_cmd(dev, LHA7878_CMD_RDATAC);
}

lha7878_status_t lha7878_exit_rdatac(lha7878_dev_t *dev)
{
    return lha7878_cmd(dev, LHA7878_CMD_SDATAC);
}

/* ------------------------------ 初始化 ---------------------------------- */
/* 写后回读校验, 失败时打印寄存器地址/写入值/回读值, 便于 bring-up 定位. */
static lha7878_status_t dbg_write_verify(lha7878_dev_t *dev, uint8_t addr, uint8_t value)
{
    lha7878_status_t st = lha7878_write_reg(dev, addr, value);
    uint8_t back = 0;
    if (st != LHA7878_OK) { return st; }
    st = lha7878_read_reg(dev, addr, &back);
    if (st != LHA7878_OK) { return st; }
    if (back != value) {
        printf("[DBG] reg 0x%02X verify fail: wrote 0x%02X, read 0x%02X\r\n",
               (unsigned)addr, (unsigned)value, (unsigned)back);
        return LHA7878_ERR_VERIFY;
    }
    return LHA7878_OK;
}

static lha7878_status_t write_config_registers(lha7878_dev_t *dev)
{
    lha7878_status_t st;
    uint8_t config1, config2, config3;
    const lha7878_config_t *c = &dev->cfg;

    /* CONFIG3 先写: 内部基准时 PDB_REFBUF=1 后等基准稳定 (图42).
     * 稳定等待时间手册未给具体值, 这里取 10ms 裕量, 不是手册值. */
    config3 = 0u;
    if (c->vref != LHA7878_VREF_EXT) { config3 |= LHA7878_CONFIG3_PDB_REFBUF; }
    if (c->vref == LHA7878_VREF_INT_4V) { config3 |= LHA7878_CONFIG3_VREF_4V; }
    st = dbg_write_verify(dev, LHA7878_REG_CONFIG3, config3);
    if (st != LHA7878_OK) { return st; }
    if (c->vref != LHA7878_VREF_EXT) { lha7878_delay_ms(10u); }

    config1 = (uint8_t)(LHA7878_CONFIG1_FIXED |
                         (c->multi_readback ? (1u << 6) : 0u) |
                         (c->clk_en ? (1u << 5) : 0u) |
                         (uint8_t)c->data_rate);
    st = dbg_write_verify(dev, LHA7878_REG_CONFIG1, config1);
    if (st != LHA7878_OK) { return st; }

    config2 = (uint8_t)(LHA7878_CONFIG2_FIXED |
                         (c->test_int ? (1u << 4) : 0u) |
                         (c->test_freq & 0x03u));
    st = dbg_write_verify(dev, LHA7878_REG_CONFIG2, config2);
    if (st != LHA7878_OK) { return st; }

    st = dbg_write_verify(dev, LHA7878_REG_FAULT,
                          (uint8_t)(c->fault_th << LHA7878_FAULT_COMP_TH_SHIFT));
    if (st != LHA7878_OK) { return st; }

    st = lha7878_write_all_channels(dev);
    if (st != LHA7878_OK) { return st; }
    /* 回读 8 个 CHnSET 校验. */
    {
        uint8_t back[8];
        st = lha7878_read_regs(dev, LHA7878_REG_CH1SET, back, 8u);
        if (st != LHA7878_OK) { return st; }
        for (uint8_t i = 0; i < 8u; i++) {
            uint8_t expect = chnset_byte(c->gain[i], c->mux[i], c->channel_pd[i]);
            if (back[i] != expect) {
                printf("[DBG] CH%uSET verify fail: wrote 0x%02X, read 0x%02X\r\n",
                       (unsigned)(i + 1u), (unsigned)expect, (unsigned)back[i]);
                return LHA7878_ERR_VERIFY;
            }
        }
    }
    /* GPIO (§9.6.1.9 表38): GPIOD[7:4] 回读的是引脚实际电平, 与方向无关,
     * 不能用整字节相等校验; 只校验可回读的方向位 GPIOC[3:0]. */
    {
        uint8_t gpio_val = (uint8_t)(((c->gpio_out & 0x0Fu) << 4) |
                                     (c->gpio_dir & 0x0Fu));
        uint8_t back = 0;
        st = lha7878_write_reg(dev, LHA7878_REG_GPIO, gpio_val);
        if (st != LHA7878_OK) { return st; }
        st = lha7878_read_reg(dev, LHA7878_REG_GPIO, &back);
        if (st != LHA7878_OK) { return st; }
        if ((back & 0x0Fu) != (c->gpio_dir & 0x0Fu)) {
            printf("[DBG] GPIO dir verify fail: wrote 0x%02X, read 0x%02X\r\n",
                   (unsigned)gpio_val, (unsigned)back);
            return LHA7878_ERR_VERIFY;
        }
    }
    return LHA7878_OK;
}

lha7878_status_t lha7878_init(lha7878_dev_t *dev)
{
    lha7878_status_t st;
    if (dev == NULL) { return LHA7878_ERR_ARG; }
    dev->initialized = false;
    dev->in_rdatac = false;

    st = lha7878_powerup(dev);
    if (st != LHA7878_OK) { return st; }

    st = write_config_registers(dev);
    if (st != LHA7878_OK) { return st; }

    /* §9.5.3.7: 增益配好后做一次失调校准.
     * 注意: OFFSETCAL 会将硬件速率强制改为 1 kSPS (DR=110b, 见 Rev.D §9.5.3.7),
     * 若未在校准后恢复 CONFIG1 会导致一直以 1 kSPS 运行. 此处暂时跳过失调校准. */
    /*
    st = lha7878_offset_cal(dev);
    if (st != LHA7878_OK) { return st; }
    */

    /* 图42: START 脚拉高开始转换, 等建立时间 (表15) + 首个 DRDY. */
    st = lha7878_start_pin(dev, true);
    if (st != LHA7878_OK) { return st; }
    {
        uint32_t settle_ms = (uint32_t)(lha7878_settle_tclk[(uint8_t)dev->cfg.data_rate] *
                                        1000u / dev->cfg.clk_hz) + 10u;
        uint32_t period_ms = 1000u / lha7878_dr_ksps[(uint8_t)dev->cfg.data_rate] + 2u;
        st = lha7878_wait_drdy(dev, settle_ms + 2u * period_ms);
        if (st != LHA7878_OK) { return st; }
    }

    if (dev->cfg.mode == LHA7878_MODE_RDATAC) {
        st = lha7878_enter_rdatac(dev);
        if (st != LHA7878_OK) { return st; }
    }
    /* RDATA 模式: 保持 SDATAC 状态, 每帧由 lha7878_read_frame_rdata() 发 RDATA. */
    dev->initialized = true;
    return LHA7878_OK;
}

/* ------------------------------ DRDY / 读帧 ------------------------------ */
bool lha7878_drdy_is_low(const lha7878_dev_t *dev)
{
    if (dev == NULL) { return false; }
    return HAL_GPIO_ReadPin(dev->drdy_port, dev->drdy_pin) == GPIO_PIN_RESET;
}

lha7878_status_t lha7878_wait_drdy(lha7878_dev_t *dev, uint32_t timeout_ms)
{
    uint32_t start;
    uint32_t wait_high_timeout;
    if (dev == NULL) { return LHA7878_ERR_ARG; }
    start = HAL_GetTick();

    /* 阶段 1: 若当前 DRDY 为低 (可能因主循环卡顿导致新帧已在就绪等待):
     * 先尝试等待其被拉高, 以便捕获下一周期的干净下降沿.
     * 注意: 未读数时芯片仅在周期末尾刷新时自动拉高 ~1.95 µs, 极易漏检.
     * 故阶段 1 最多等 2ms (约 16 个采样周期), 若仍为低电平说明移位寄存器中已有就绪帧,
     * 此时直接返回 OK 兜底放行读取! 发出的 SCLK 会立刻将 DRDY 拉高, 从而瞬间解除自锁并在下一周期重同步. */
    wait_high_timeout = (timeout_ms < 2u) ? timeout_ms : 2u;
    while (lha7878_drdy_is_low(dev)) {
        if ((HAL_GetTick() - start) >= wait_high_timeout) {
            return LHA7878_OK;  /* 兜底破自锁: 直接放行就绪帧读取, 发 SCLK 清高 DRDY */
        }
    }

    /* 阶段 2: 等待新周期的 DRDY 下降沿(新采样数据准备就绪的瞬间) */
    while (!lha7878_drdy_is_low(dev)) {
        if ((HAL_GetTick() - start) >= timeout_ms) {
            return LHA7878_ERR_TIMEOUT;
        }
    }

    return LHA7878_OK;
}

lha7878_status_t lha7878_parse_frame(const uint8_t *buf, lha7878_frame_t *frame,
                                     uint32_t frame_index)
{
    uint32_t status;
    if (buf == NULL || frame == NULL) { return LHA7878_ERR_ARG; }
    status = ((uint32_t)buf[0] << 16) | ((uint32_t)buf[1] << 8) | (uint32_t)buf[2];
    /* 图35: 状态字前 4 位固定为 1100b. 原版只判 (buf[0]&0xC0)==0xC0,
     * 这里判满 4 位, 更严. */
    if ((status >> 20) != 0xCu) { return LHA7878_ERR_FRAME; }
    frame->status_raw = status;
    frame->fault_p = (uint8_t)((status >> 12) & 0xFFu);
    frame->fault_n = (uint8_t)((status >> 4) & 0xFFu);
    frame->gpio = (uint8_t)(status & 0x0Fu);
    frame->frame_index = frame_index;
    for (uint8_t i = 0; i < 8u; i++) {
        int32_t v = (int32_t)(((uint32_t)buf[3u + 3u * i] << 16) |
                              ((uint32_t)buf[4u + 3u * i] << 8) |
                              (uint32_t)buf[5u + 3u * i]);
        if ((v & 0x00800000) != 0) { v |= (int32_t)0xFF000000; }  /* 24 位补码符号扩展 */
        frame->raw[i] = v;
    }
    return LHA7878_OK;
}

lha7878_status_t lha7878_read_frame_rdatac(lha7878_dev_t *dev, lha7878_frame_t *frame,
                                           uint32_t timeout_ms)
{
    lha7878_status_t st;
    uint8_t tx[LHA7878_FRAME_BYTES] = { 0 };
    uint8_t rx[LHA7878_FRAME_BYTES] = { 0 };
    if (dev == NULL || frame == NULL || !dev->initialized || !dev->in_rdatac) {
        return LHA7878_ERR_NOT_READY;
    }
    st = lha7878_wait_drdy(dev, timeout_ms);
    if (st != LHA7878_OK) { return st; }
    cs_assert(dev);
    /* DIN 全发 0x00, 216 个 SCLK, 第一个 SCLK 下降沿 DRDY 回高 (§9.4.5.2). */
    st = spi_xfer_buf(dev, tx, rx, LHA7878_FRAME_BYTES);
    cs_release(dev);
    if (st != LHA7878_OK) { dev->frame_error_count++; return st; }
    st = lha7878_parse_frame(rx, frame, dev->frame_count);
    if (st != LHA7878_OK) { dev->frame_error_count++; return st; }
    dev->frame_count++;
    /* 故障位不丢帧: 帧已填好, 但返回 ERR_FAULT 让调用方决定是否停机. */
    return (frame->fault_p != 0u || frame->fault_n != 0u) ? LHA7878_ERR_FAULT : LHA7878_OK;
}

lha7878_status_t lha7878_read_frame_rdata(lha7878_dev_t *dev, lha7878_frame_t *frame,
                                          uint32_t timeout_ms)
{
    lha7878_status_t st;
    /* 1 字节 RDATA 命令 + 27 字节数据, 同一个 CS 周期内连续发完.
     * RDATA 后取数 SCLK 不需要等待 (§9.5.3.10). */
    uint8_t tx[LHA7878_FRAME_BYTES + 1u] = { 0 };
    uint8_t rx[LHA7878_FRAME_BYTES + 1u] = { 0 };
    if (dev == NULL || frame == NULL || !dev->initialized || dev->in_rdatac) {
        return LHA7878_ERR_NOT_READY;
    }
    st = lha7878_wait_drdy(dev, timeout_ms);
    if (st != LHA7878_OK) { return st; }
    tx[0] = LHA7878_CMD_RDATA;
    cs_assert(dev);
    st = spi_xfer_buf(dev, tx, rx, LHA7878_FRAME_BYTES + 1u);
    cs_release(dev);
    if (st != LHA7878_OK) { dev->frame_error_count++; return st; }
    st = lha7878_parse_frame(&rx[1], frame, dev->frame_count);
    if (st != LHA7878_OK) { dev->frame_error_count++; return st; }
    dev->frame_count++;
    return (frame->fault_p != 0u || frame->fault_n != 0u) ? LHA7878_ERR_FAULT : LHA7878_OK;
}

lha7878_status_t lha7878_read_frame(lha7878_dev_t *dev, lha7878_frame_t *frame,
                                    uint32_t timeout_ms)
{
    if (dev == NULL) { return LHA7878_ERR_ARG; }
    return (dev->cfg.mode == LHA7878_MODE_RDATA)
               ? lha7878_read_frame_rdata(dev, frame, timeout_ms)
               : lha7878_read_frame_rdatac(dev, frame, timeout_ms);
}

/* ------------------------------ 换算 ------------------------------------ */
int64_t lha7878_code_to_uv(int32_t code, uint32_t vref_uv, lha7878_gain_t gain)
{
    uint8_t g = (uint8_t)gain;
    if (g == 0u) { g = 1u; }
    /* V = code * 2*VREF/Gain / 2^24 (§9.5.1 公式8). int64 防溢出. */
    return ((int64_t)code * 2LL * (int64_t)vref_uv / (int64_t)g) / 16777216LL;
}

int32_t lha7878_code_to_mv(int32_t code, uint32_t vref_uv, lha7878_gain_t gain)
{
    return (int32_t)(lha7878_code_to_uv(code, vref_uv, gain) / 1000LL);
}

float lha7878_code_to_volt(int32_t code, uint32_t vref_uv, lha7878_gain_t gain)
{
    return (float)lha7878_code_to_uv(code, vref_uv, gain) / 1000000.0f;
}

void lha7878_frame_to_uv(const lha7878_dev_t *dev, const lha7878_frame_t *frame, int64_t *uv)
{
    uint32_t vref;
    if (dev == NULL || frame == NULL || uv == NULL) { return; }
    vref = lha7878_vref_uv(dev);
    for (uint8_t i = 0; i < 8u; i++) {
        uv[i] = lha7878_code_to_uv(frame->raw[i], vref, dev->cfg.gain[i]);
    }
}

int32_t lha7878_temp_centi_c(int32_t code, uint32_t vref_uv)
{
    int64_t uv = lha7878_code_to_uv(code, vref_uv, LHA7878_GAIN_1);
    /* §9.3.2.3 公式 (3): T = (Vuv-145300)/490 + 25, 增益须为 1. */
    return (int32_t)(((uv - 145300LL) * 100LL) / 490LL + 2500LL);
}

int32_t lha7878_mvdd_mv(int32_t code, uint32_t vref_uv, uint8_t ch,
                         uint32_t avdd_mv, uint32_t dvdd_mv)
{
    int32_t mv;
    (void)avdd_mv; (void)dvdd_mv;  /* 换算只依赖实测电压, 电源值仅作调用方核对用 */
    if (ch < 1u || ch > 8u) { return 0; }  /* ch 为 1~8 的通道号, 不是下标 */
    mv = lha7878_code_to_mv(code, vref_uv, LHA7878_GAIN_1);
    /* §9.3.2.4: CH1/2/5/6/7/8 量 0.5*(AVDD-AVSS), CH3/4 量 DVDD/4. */
    if (ch == 3u || ch == 4u) { return mv * 4; }
    return mv * 2;
}

/* ------------------------------ 故障 / GPIO ------------------------------ */
lha7878_status_t lha7878_read_fault_status(lha7878_dev_t *dev, uint8_t *fault_p, uint8_t *fault_n)
{
    uint8_t buf[2];
    lha7878_status_t st;
    if (dev == NULL || fault_p == NULL || fault_n == NULL) { return LHA7878_ERR_ARG; }
    st = lha7878_read_regs(dev, LHA7878_REG_FAULT_STATP, buf, 2u);
    if (st != LHA7878_OK) { return st; }
    *fault_p = buf[0];
    *fault_n = buf[1];
    return LHA7878_OK;
}

lha7878_status_t lha7878_set_fault_threshold(lha7878_dev_t *dev, uint8_t comp_th)
{
    if (dev == NULL || comp_th > 7u) { return LHA7878_ERR_ARG; }
    dev->cfg.fault_th = comp_th;
    return lha7878_write_reg_verify(dev, LHA7878_REG_FAULT,
                                    (uint8_t)(comp_th << LHA7878_FAULT_COMP_TH_SHIFT));
}

lha7878_status_t lha7878_gpio_read(lha7878_dev_t *dev, uint8_t *gpiod)
{
    uint8_t v = 0;
    lha7878_status_t st;
    if (dev == NULL || gpiod == NULL) { return LHA7878_ERR_ARG; }
    st = lha7878_read_reg(dev, LHA7878_REG_GPIO, &v);
    if (st != LHA7878_OK) { return st; }
    *gpiod = (uint8_t)((v >> 4) & 0x0Fu);
    return LHA7878_OK;
}

lha7878_status_t lha7878_gpio_write(lha7878_dev_t *dev, uint8_t gpiod_out, uint8_t gpioc_dir)
{
    uint8_t v;
    if (dev == NULL) { return LHA7878_ERR_ARG; }
    dev->cfg.gpio_out = (uint8_t)(gpiod_out & 0x0Fu);
    dev->cfg.gpio_dir = (uint8_t)(gpioc_dir & 0x0Fu);
    v = (uint8_t)(((dev->cfg.gpio_out & 0x0Fu) << 4) | (dev->cfg.gpio_dir & 0x0Fu));
    /* GPIOD[7:4] 回读为引脚电平, 只校验方向位 GPIOC[3:0] (§9.6.1.9). */
    lha7878_status_t st = lha7878_write_reg(dev, LHA7878_REG_GPIO, v);
    uint8_t back = 0;
    if (st != LHA7878_OK) { return st; }
    st = lha7878_read_reg(dev, LHA7878_REG_GPIO, &back);
    if (st != LHA7878_OK) { return st; }
    return ((back & 0x0Fu) == (dev->cfg.gpio_dir & 0x0Fu)) ? LHA7878_OK : LHA7878_ERR_VERIFY;
}

/* ------------------------------ 自检 ------------------------------------ */
static lha7878_status_t selftest_run(lha7878_dev_t *dev, uint8_t config2,
                                     uint8_t chnset, lha7878_frame_t *frame)
{
    lha7878_status_t st;
    uint8_t saved_config2 = 0, saved_chn[8] = { 0 }, new_chn[8];
    uint32_t period_ms;
    if (dev == NULL || frame == NULL || !dev->initialized) { return LHA7878_ERR_NOT_READY; }

    st = lha7878_read_reg(dev, LHA7878_REG_CONFIG2, &saved_config2);
    if (st != LHA7878_OK) { return st; }
    st = lha7878_read_regs(dev, LHA7878_REG_CH1SET, saved_chn, 8u);
    if (st != LHA7878_OK) { return st; }

    st = lha7878_exit_rdatac(dev);  /* RDATAC 下 RREG/WREG 被忽略, 先退出 */
    if (st != LHA7878_OK) { return st; }
    for (uint8_t i = 0; i < 8u; i++) { new_chn[i] = chnset; }
    st = lha7878_write_reg_verify(dev, LHA7878_REG_CONFIG2, config2);
    if (st == LHA7878_OK) { st = lha7878_write_regs(dev, LHA7878_REG_CH1SET, new_chn, 8u); }
    if (st != LHA7878_OK) { goto restore; }

    /* 输入阶跃后滤波器需 3*tDR 建立, 第 4 个 DRDY 才稳定 (§9.4.1.2).
     * DRDY 在未读数时会一直保持低, 不能靠数 DRDY 边沿计时, 这里按
     * 时间等满 4 个周期再读, 不要拿第一帧当结论. */
    period_ms = 1000u / lha7878_dr_ksps[(uint8_t)dev->cfg.data_rate] + 2u;
    st = lha7878_enter_rdatac(dev);
    if (st != LHA7878_OK) { goto restore; }
    lha7878_delay_ms(4u * period_ms);
    st = lha7878_read_frame_rdatac(dev, frame, period_ms * 2u);
    if (st != LHA7878_OK) { goto restore; }

restore:
    /* 恢复原配置, 不论自检成败都要恢复, 否则会把测试 MUX 留在线上. */
    {
        lha7878_status_t rst = lha7878_exit_rdatac(dev);
        if (rst == LHA7878_OK) { rst = lha7878_write_reg_verify(dev, LHA7878_REG_CONFIG2, saved_config2); }
        if (rst == LHA7878_OK) { rst = lha7878_write_regs(dev, LHA7878_REG_CH1SET, saved_chn, 8u); }
        if (rst == LHA7878_OK && dev->cfg.mode == LHA7878_MODE_RDATAC) {
            rst = lha7878_enter_rdatac(dev);
        }
        if (st == LHA7878_OK) { st = rst; }
    }
    return st;
}

lha7878_status_t lha7878_selftest_short(lha7878_dev_t *dev, lha7878_frame_t *frame)
{
    /* MUX=001 短接至中点, 增益 1 (§9.3.2.1 噪声测量). 读数应接近 0,
     * 具体噪声上限按 §8.1 表10/12 与你的速率/增益查, 驱动不硬编码. */
    if (dev == NULL) { return LHA7878_ERR_ARG; }
    return selftest_run(dev,
                        (uint8_t)(LHA7878_CONFIG2_FIXED |
                                  (dev->cfg.test_int ? (1u << 4) : 0u) |
                                  (dev->cfg.test_freq & 0x03u)),
                        chnset_byte(LHA7878_GAIN_1, LHA7878_MUX_SHORT, false), frame);
}

lha7878_status_t lha7878_selftest_signal(lha7878_dev_t *dev, lha7878_frame_t *frame)
{
    /* CONFIG2: INT_TEST=1, TEST_FREQ=01 (fCLK/2^20), 即 0xD1;
     * CHnSET: 增益 1, MUX=101 测试信号, 即 0x15 (图42). */
    if (dev == NULL) { return LHA7878_ERR_ARG; }
    return selftest_run(dev,
                        (uint8_t)(LHA7878_CONFIG2_FIXED | (1u << 4) |
                                  (uint8_t)LHA7878_TEST_FREQ_DIV20),
                        chnset_byte(LHA7878_GAIN_1, LHA7878_MUX_TEST, false), frame);
}
