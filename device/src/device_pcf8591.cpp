#include "device_pcf8591.hpp"

// ============================================================
//  来源：Rob Tillaart PCF8591 v0.4.2
//  移植说明见 device_pcf8591.hpp 文件头
//
//  读时序说明（数据手册 P8）：
//   读回的第一个字节是上一次转换的结果，必须丢弃。
//   freedom_read(reg = 控制字节, &data, 2) 一次完成：
//   写控制字节（选择通道/模式）→ restart → 读 2 字节，
//   结果大端组装：data = [前次结果][当前通道值]，取低字节。
// ============================================================

// ============================================================
//  构造 / 初始化
// ============================================================

PCF8591::PCF8591(inter_i2c_bus* bus, uint8_t addr)
    : _dev(bus, addr)
    , _addr(addr)
    , _control(0)
    , _dac(0)
    , _error(ERR_OK)
{
    for (uint8_t i = 0; i < 4; i++) _adc[i] = 0;
}

void PCF8591::init()
{
    if (!isConnected()) return;
    (void)write(0);   // 对齐原版 begin(0)：初始 DAC 值 0
}

bool PCF8591::isConnected()
{
    return _dev.ping();
}

uint8_t PCF8591::getAddress()
{
    return _addr;
}

// ============================================================
//  ADC 部分
// ============================================================

void PCF8591::enableINCR()
{
    _control |= 0x04;   // bit2: INCR 地址自增
}

void PCF8591::disableINCR()
{
    _control &= ~0x04;
}

bool PCF8591::isINCREnabled()
{
    return (_control & 0x04) != 0;
}

uint8_t PCF8591::read(uint8_t channel, uint8_t mode)
{
    // 参数校验
    if (mode > TWO_DIFFERENTIAL)
    {
        _error = ERR_MODE;
        return 0;
    }

    // 保留 DAC 使能 / INCR 标志，重设模式位
    _control &= 0x44;                       // bit6 DAC + bit2 INCR
    _control |= static_cast<uint8_t>(mode << 4);

    // 通道范围随模式变化（数据手册图 4）
    _error = ERR_CHANNEL;
    switch (mode)
    {
        case FOUR_SINGLE_CHANNEL: if (channel > 3) return 0; break;
        case THREE_DIFFERENTIAL:  if (channel > 2) return 0; break;
        case MIXED:               if (channel > 2) return 0; break;
        case TWO_DIFFERENTIAL:    if (channel > 1) return 0; break;
        default: return 0;
    }
    _control |= channel;
    _error = ERR_OK;

    // 写控制字节 + 读 2 字节（第 1 字节 = 前次结果，丢弃）
    uint64_t data = 0;
    if (!_dev.freedom_read(_control, &data, 2))
    {
        _error = ERR_I2C;
        return _adc[channel];   // 读失败：返回上次已知值
    }
    _adc[channel] = static_cast<uint8_t>(data & 0xFF);
    return _adc[channel];
}

uint8_t PCF8591::read4()
{
    // 4 通道连读（INCR）：写控制字节 + 读 5 字节
    _control &= 0x44;               // 模式清零，保持 DAC/INCR 标志
    _control |= 0x00;               // 通道 0 + 模式 0
    _control |= 0x04;               // INCR

    uint64_t data = 0;
    bool ok = _dev.freedom_read(_control, &data, 5);
    _control &= ~0x04;              // 关闭 INCR（对齐原版）

    if (!ok)
    {
        _error = ERR_I2C;
        return _error;
    }
    // data 大端：[前次结果][ch0][ch1][ch2][ch3]
    _adc[0] = static_cast<uint8_t>((data >> 24) & 0xFF);
    _adc[1] = static_cast<uint8_t>((data >> 16) & 0xFF);
    _adc[2] = static_cast<uint8_t>((data >> 8)  & 0xFF);
    _adc[3] = static_cast<uint8_t>(data & 0xFF);
    _error = ERR_OK;
    return _error;
}

uint8_t PCF8591::lastRead(uint8_t channel)
{
    if (channel > 3)
    {
        _error = ERR_CHANNEL;
        return 0;
    }
    return _adc[channel];
}

// 差分模式读数：8-bit 补码按 int8_t 符号解释（原版实验性 API）
int PCF8591::readComparator_01() { return static_cast<int8_t>(read(0, TWO_DIFFERENTIAL)); }
int PCF8591::readComparator_23() { return static_cast<int8_t>(read(1, TWO_DIFFERENTIAL)); }
int PCF8591::readComparator_03() { return static_cast<int8_t>(read(0, THREE_DIFFERENTIAL)); }
int PCF8591::readComparator_13() { return static_cast<int8_t>(read(1, THREE_DIFFERENTIAL)); }

// ============================================================
//  DAC 部分
// ============================================================

void PCF8591::enableDAC()
{
    _control |= 0x40;   // bit6: DAC 模拟输出使能
}

void PCF8591::disableDAC()
{
    _control &= ~0x40;
}

bool PCF8591::isDACEnabled()
{
    return (_control & 0x40) != 0;
}

bool PCF8591::write(uint8_t value)
{
    // 写控制字节 + DAC 值（2 字节事务）
    _dev.freedom_write(_control, value, 1);
    if (_dev.lastError() != 0)
    {
        _error = ERR_I2C;
        return false;
    }
    _dac = value;
    return true;
}

uint8_t PCF8591::lastWrite()
{
    return _dac;
}

// ============================================================
//  错误
// ============================================================

int PCF8591::lastError()
{
    int e = _error;
    _error = ERR_OK;
    return e;
}
