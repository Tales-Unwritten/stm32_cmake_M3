#include "inter_soft_spi.hpp"

// ============================================================
//  构造 / 析构
// ============================================================

soft_spi_bus::soft_spi_bus(const SoftSpiConfig &cfg)
    : _sck(cfg.sck_port, cfg.sck_pin), _mosi(cfg.mosi_port, cfg.mosi_pin), _miso(cfg.miso_port, cfg.miso_pin),
      _cs(cfg.cs_port, cfg.cs_pin), _mode(cfg.mode), _bit_order(cfg.bit_order), _delay_us(cfg.delay_us),
      _cpol((cfg.mode >> 1) & 0x01) // CPOL：SCK 空闲电平
      ,
      _cpha(cfg.mode & 0x01) // CPHA：采样沿
      ,
      _cs_active_level(cfg.cs_active_level), _has_cs(cfg.cs_port != nullptr && cfg.cs_pin != pin_none),
      _initialized(false), _busy(0)
{
}

soft_spi_bus::~soft_spi_bus()
{
    deinit();
}

// ============================================================
//  生命周期
// ============================================================

void soft_spi_bus::init()
{
    if (_initialized)
        return; // 幂等

    // SCK: 推挽输出，先置空闲电平（= CPOL）
    _sck.init(mode_out_pp, nopull, speed_high);
    _sck.set(_cpol ? true : false);

    // MOSI: 推挽输出
    _mosi.init(mode_out_pp, nopull, speed_high);
    _mosi.set(false);

    // MISO: 输入（从机驱动，无需上拉）
    _miso.init(mode_input, nopull);

    // CS: 推挽输出（可选），释放到无效电平
    if (_has_cs)
    {
        _cs.init(mode_out_pp, nopull, speed_high);
        cs_deselect();
    }

    _initialized = true;
}

void soft_spi_bus::deinit()
{
    if (!_initialized)
        return;

    _sck.deinit();
    _mosi.deinit();
    _miso.deinit();
    if (_has_cs)
        _cs.deinit();

    _initialized = false;
    _busy = 0;
}

// ============================================================
//  延时（0 = 空操作，速率由 io_ctrl 位操作开销决定）
// ============================================================

void soft_spi_bus::_delay()
{
    if (_delay_us != 0)
        delay_us(_delay_us);
}

// ============================================================
//  位循环核心（统一 4 种模式，无需分支复制）
//
//  每 bit 周期：
//    MOSI 数据先就绪 → 第一沿(SCK 翻转) → [CPHA=0 在此采样]
//    → 第二沿(SCK 回空闲) → [CPHA=1 在此采样]
//
//  验证：
//    MODE0(CPOL0/CPHA0): 上升沿采样 ✓  MOSI 先稳定 ✓
//    MODE1(CPOL0/CPHA1): 下降沿采样 ✓
//    MODE2(CPOL1/CPHA0): 下降沿采样 ✓
//    MODE3(CPOL1/CPHA1): 上升沿采样 ✓
// ============================================================

uint8_t soft_spi_bus::_transfer_byte_core(uint8_t data)
{
    uint8_t rx = 0;
    bool msb = (_bit_order == MSB);

    for (uint8_t i = 0; i < 8; i++)
    {
        // 1. MOSI 数据就绪（采样沿之前稳定）
        _mosi.set(msb ? ((data & 0x80) != 0) : ((data & 0x01) != 0));
        _delay();

        // 2. 第一沿（进入非空闲电平）
        _sck.set(!_cpol);
        _delay();

        // 3. CPHA=0：第一沿采样 MISO
        bool rx_bit = false;
        if (!_cpha)
            rx_bit = (_miso.read() == Hig);

        // 4. 第二沿（回到空闲电平）
        _sck.set(_cpol);
        _delay();

        // 5. CPHA=1：第二沿采样 MISO
        if (_cpha)
            rx_bit = (_miso.read() == Hig);

        // 6. 移位（位序决定方向）
        if (msb)
        {
            rx <<= 1;
            if (rx_bit)
                rx |= 0x01;
            data <<= 1;
        }
        else
        {
            rx >>= 1;
            if (rx_bit)
                rx |= 0x80;
            data >>= 1;
        }
    }
    return rx;
}

// ============================================================
//  数据传输
// ============================================================

uint8_t soft_spi_bus::transfer_byte(uint8_t data)
{
    return _transfer_byte_core(data);
}

void soft_spi_bus::transfer(const uint8_t *tx, uint8_t *rx, uint16_t len)
{
    if (len == 0)
        return;

    for (uint16_t i = 0; i < len; i++)
    {
        uint8_t out = tx ? tx[i] : 0xFF;
        uint8_t in = _transfer_byte_core(out);
        if (rx)
            rx[i] = in;
    }
}

// ============================================================
//  片选（内部 CS 模式）
// ============================================================

void soft_spi_bus::cs_select()
{
    if (!_has_cs || !_initialized)
        return;
    _cs.write(_cs_active_level); // 选中 = 输出有效电平
}

void soft_spi_bus::cs_deselect()
{
    if (!_has_cs || !_initialized)
        return;
    _cs.write(_cs_active_level == active_low ? Hig : Low); // 释放 = 输出无效电平
}

// ============================================================
//  运行时配置
// ============================================================

// ============================================================
//  SPI 模式说明（mode 0~3 = CPOL + CPHA 两个独立参数的组合）
//
//  参数定义：
//    CPOL —— SCK 空闲电平：0 = 空闲低，1 = 空闲高
//    CPHA —— 采样沿选择：  0 = 第一沿采样，1 = 第二沿采样
//
//  四模式对比（每 bit 时钟翻转两次，采样发生在其中一个沿）：
//    mode 0: CPOL=0 CPHA=0 → 空闲低，上升沿(第一沿)采样
//    mode 1: CPOL=0 CPHA=1 → 空闲低，下降沿(第二沿)采样
//    mode 2: CPOL=1 CPHA=0 → 空闲高，下降沿(第一沿)采样
//    mode 3: CPOL=1 CPHA=1 → 空闲高，上升沿(第二沿)采样
//
//  为什么需要模式：
//    主从双方必须使用完全相同的模式，否则数据错位（典型症状：
//    读 ID 全 FF/00、寄存器读回乱码）。不同器件要求不同：
//    - W25Q 系列 SPI Flash：支持 MODE0 / MODE3
//    - SD 卡（SPI 模式）：通常要求 MODE0
//    - 部分 ADC/DAC：只支持 MODE1 或 MODE2
//    换器件时查数据手册的 CPOL/CPHA 章节，将 mode 设为器件要求的值。
//
//  实现对应：mode 拆分为 _cpol（SCK 空闲电平）与 _cpha（采样沿），
//    传输循环中：CPHA=0 在 SCK 第一沿采样，CPHA=1 在第二沿采样。
// ============================================================

void soft_spi_bus::set_mode(soft_mode_enum_t mode)
{
    _mode = mode;
    _cpol = (mode >> 1) & 0x01;
    _cpha = mode & 0x01;

    // 立即将 SCK 置于新模式的空闲电平
    if (_initialized)
        _sck.set(_cpol ? true : false);
}

void soft_spi_bus::set_bit_order(soft_order_enum_t order)
{
    _bit_order = order;
}

void soft_spi_bus::set_speed(uint8_t delay_us)
{
    _delay_us = delay_us;
}

// ============================================================
//  互斥（IRQ-safe spinlock，对齐 inter_i2c_bus）
// ============================================================

void soft_spi_bus::lock()
{
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    while (_busy)
    {
        __set_PRIMASK(mask); // 短暂恢复中断，让 ISR 运行
        __disable_irq();
    }
    _busy = 1;
    __set_PRIMASK(mask);
}

void soft_spi_bus::unlock()
{
    _busy = 0;
}
