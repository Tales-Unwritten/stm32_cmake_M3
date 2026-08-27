// ============================================================
// @platform GD32F4xx（当前平台）
//   移植到新 MCU 时，本文件需要完整重写，但公共接口签名不变。
//   需要替换的映射：
//     - 外设基址:   I2C0/I2C1/I2C2 → 目标平台外设基址
//     - 时钟使能:   rcu_periph_clock_enable(RCU_I2Cx)
//     - 外设初始化:  i2c_clock_config / i2c_mode_addr_config
//     - 总线操作:   I2C_CTL0_START/STOP, I2C_DATA, I2C_STAT0 flags
//     - GPIO 复用:  AF4 → 目标平台 AF 编号
// ============================================================

#include "inter_i2c_hw.hpp"

#include "gd32f4xx.h"
#include "gd32f4xx_rcu.h"
#include "gd32f4xx_i2c.h"

// ============================================================
//  构造 / 析构
// ============================================================

i2c_hw_port::i2c_hw_port(const I2cHwConfig &cfg)
    : _cfg(cfg)
    , _scl(cfg.scl_port, cfg.scl_pin)
    , _sda(cfg.sda_port, cfg.sda_pin)
    , _initialized(false)
    , _periph(0)
    , _busy(0)
{
}

i2c_hw_port::~i2c_hw_port()
{
    deinit();
}

// ============================================================
//  时钟使能
// ============================================================

void i2c_hw_port::_enable_clock()
{
    // [PORT] GD32 时钟映射
    switch (_cfg.periph) {
        case i2c_hw_id::i2c0: rcu_periph_clock_enable(RCU_I2C0); break;
        case i2c_hw_id::i2c1: rcu_periph_clock_enable(RCU_I2C1); break;
        case i2c_hw_id::i2c2: rcu_periph_clock_enable(RCU_I2C2); break;
    }
}

// ============================================================
//  init
// ============================================================

void i2c_hw_port::init()
{
    if (_initialized) return;

    // [PORT] GD32 外设基址映射
    switch (_cfg.periph) {
        case i2c_hw_id::i2c0: _periph = I2C0; break;
        case i2c_hw_id::i2c1: _periph = I2C1; break;
        case i2c_hw_id::i2c2: _periph = I2C2; break;
    }

    _enable_clock();

    // GPIO: 开漏输出 + 内部上拉，AF 模式
    _scl.init(mode_af_od, pullup, speed_high);
    _scl.set_af(_cfg.af);
    _sda.init(mode_af_od, pullup, speed_high);
    _sda.set_af(_cfg.af);

    // I2C 时钟 + 主机模式
    i2c_clock_config(_periph, _cfg.clock_speed, I2C_DTCY_2);
    i2c_mode_addr_config(_periph, I2C_I2CMODE_ENABLE,
                         I2C_ADDFORMAT_7BITS, 0);
    i2c_enable(_periph);
    i2c_ack_config(_periph, I2C_ACK_ENABLE);

    _initialized = true;
}

// ============================================================
//  deinit
// ============================================================

void i2c_hw_port::deinit()
{
    if (!_initialized) return;

    i2c_ack_config(_periph, I2C_ACK_DISABLE);
    i2c_disable(_periph);
    i2c_deinit(_periph);

    _scl.deinit();
    _sda.deinit();

    // 禁用 I2C 时钟，防止干扰后续软 I2C
    switch (_cfg.periph) {
        case i2c_hw_id::i2c0: rcu_periph_clock_disable(RCU_I2C0); break;
        case i2c_hw_id::i2c1: rcu_periph_clock_disable(RCU_I2C1); break;
        case i2c_hw_id::i2c2: rcu_periph_clock_disable(RCU_I2C2); break;
    }

    _periph = 0;
    _initialized = false;
}

// ============================================================
//  互斥
// ============================================================

void i2c_hw_port::lock()
{
    while (_busy);
    _busy = 1;
}

void i2c_hw_port::unlock()
{
    _busy = 0;
}

// ============================================================
//  等待指定标志
// ============================================================

void i2c_hw_port::_wait_flag(i2c_flag_enum flag)
{
    uint32_t timeout = 0xFFFF;
    while (!i2c_flag_get(_periph, flag)) {
        if (--timeout == 0) return;
    }
}

// ============================================================
//  总线控制（GD32 寄存器级实现）
// ============================================================

void i2c_hw_port::start()
{
    // 等待总线空闲
    uint32_t timeout = 0xFFFF;
    while (i2c_flag_get(_periph, I2C_FLAG_I2CBSY)) {
        if (--timeout == 0) return;
    }
    // 生成 START（或重复 START）
    start_on_bus(_periph);
    _wait_flag(I2C_FLAG_SBSEND);
}

void i2c_hw_port::i2c_restart()
{
    // 重复 START：总线已被本主机占有，直接发 START
    start_on_bus(_periph);
    _wait_flag(I2C_FLAG_SBSEND);
}

void i2c_hw_port::stop()
{
    stop_on_bus(_periph);
    // 等待 STOP 条件已发送完成标志
    uint32_t timeout = 0xFFFF;
    while (!i2c_flag_get(_periph, I2C_FLAG_STPDET)) {
        if (--timeout == 0) return;
    }
    i2c_flag_clear(_periph, I2C_FLAG_STPDET);
}

void i2c_hw_port::write_byte(uint8_t data)
{
    _wait_flag(I2C_FLAG_TBE);
    i2c_data_transmit(_periph, data);
}

uint8_t i2c_hw_port::read_byte()
{
    _wait_flag(I2C_FLAG_RBNE);
    return i2c_data_receive(_periph);
}

bool i2c_hw_port::wait_ack(uint16_t timeout)
{
    // 等待 ADDSEND 或超时（主设备地址发送后从机应答）
    while (!i2c_flag_get(_periph, I2C_FLAG_ADDSEND)) {
        if (--timeout == 0) return false;
    }
    i2c_flag_clear(_periph, I2C_FLAG_ADDSEND);
    return true;
}

void i2c_hw_port::write_ack(uint8_t ack)
{
    i2c_ack_config(_periph, ack ? I2C_ACK_DISABLE : I2C_ACK_ENABLE);
}

// ============================================================
//  设备级操作（基于总线 API 实现，与 inter_i2c_dev 一致）
// ============================================================

void i2c_hw_port::i2c_write_reg(uint8_t dev_addr_7bit, uint8_t reg,
                                 const uint8_t *data, uint16_t len)
{
    lock();

    start();
    write_byte(dev_addr_7bit << 1);   // 7-bit → 8-bit 写地址
    if (!wait_ack()) goto exit;        // 地址阶段：等 ADDSEND
    write_byte(reg);                   // 寄存器地址
    // 数据字节后不等 ACK（ADDSEND 只在地址阶段有效）

    for (uint16_t i = 0; i < len; i++) {
        write_byte(data[i]);
        // 数据阶段 NACK 检测：从机不应答则中止
        if (i2c_flag_get(_periph, I2C_FLAG_AERR)) {
            i2c_flag_clear(_periph, I2C_FLAG_AERR);
            goto exit;
        }
    }

exit:
    stop();
    unlock();
}

void i2c_hw_port::i2c_read_reg(uint8_t dev_addr_7bit, uint8_t reg,
                                uint8_t *data, uint16_t len)
{
    uint8_t addr_w = dev_addr_7bit << 1;
    uint8_t addr_r = addr_w | 0x01;

    lock();

    // 阶段 1：写寄存器地址
    start();
    write_byte(addr_w);
    if (!wait_ack()) goto exit;    // 地址阶段：等 ADDSEND
    write_byte(reg);               // 寄存器地址（数据阶段，不等 ACK）

    // 阶段 2：重复 START + 读
    i2c_restart();
    write_byte(addr_r);
    if (!wait_ack()) goto exit;

    // 读取数据（单字节提前 NACK）
    if (len == 1) write_ack(1);
    for (uint16_t i = 0; i < len; i++) {
        if (i == len - 2) write_ack(1);
        data[i] = read_byte();
    }

exit:
    stop();
    write_ack(0);  // 恢复 ACK
    unlock();
}
