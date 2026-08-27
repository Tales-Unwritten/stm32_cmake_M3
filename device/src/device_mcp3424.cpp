#include "device_mcp3424.hpp"

// ============================================================
//  来源：Rob Tillaart MCP3424 v0.2.0
//  移植说明见 device_mcp3424.hpp 文件头
//
//  电压换算（定点，无浮点）：
//   数据手册：18-bit LSB = 15.625 µV，分辨率每降 2 位 LSB ×4
//     LSB_µV = 15.625 × 2^(18-bits)
//   1/8 µV 定点：LSB_x8 = 125 << (18-bits)（各分辨率均整除）：
//     18bit: 125/8 = 15.625 ✓   16bit: 500/8 = 62.5 ✓
//     14bit: 2000/8 = 250 ✓     12bit: 8000/8 = 1000 ✓
//   readVoltage_uV = raw × LSB_x8 / (8 × gain)
//     raw 最大 ±131072(18bit) × 125 = 1.6e7，int32 全程不溢出
// ============================================================

// ============================================================
//  构造 / 初始化
// ============================================================

MCP3424::MCP3424(inter_i2c_bus* bus, uint8_t addr)
    : _dev(bus, addr)
    , _bus(bus)
    , _addr(addr)
    , _channel(0)
    , _gain(1)
    , _bits(12)
    , _config(0x10)      // 连续模式 + 12bit + x1 + 通道 0
    , _raw(0)
{
}

void MCP3424::init()
{
    if (!isConnected()) return;
    (void)writeConfig();   // 写入当前配置（默认连续模式，立即开始转换）
}

bool MCP3424::isConnected()
{
    return _dev.ping();
}

uint8_t MCP3424::getAddress()
{
    return _addr;
}

uint8_t MCP3424::getMaxChannels()
{
    return 4;
}

uint8_t MCP3424::lastError()
{
    return _dev.lastError();
}

// ============================================================
//  数据读取
// ============================================================

int32_t MCP3424::read()
{
    return readRaw();
}

void MCP3424::requestSingleShot()
{
    setSingleShotMode();
}

bool MCP3424::isReady()
{
    (void)readRaw();
    return (_config & 0x80) == 0x00;   // RDY 位：0 = 转换完成
}

int32_t MCP3424::readVoltage_uV()
{
    int32_t raw = read();
    // LSB_x8 = 125 << (18-bits)，uV = raw × LSB_x8 / (8 × gain)
    int32_t lsb_x8 = 125 << (18 - _bits);
    return (raw * lsb_x8) / (8 * _gain);
}

int32_t MCP3424::readVoltage_mV()
{
    return readVoltage_uV() / 1000;
}

// ============================================================
//  配置
// ============================================================

bool MCP3424::setChannel(uint8_t channel)
{
    if (channel >= getMaxChannels()) return false;
    if (_channel != channel)
    {
        _channel = channel;
        _config &= 0x1F;                       // 清通道位
        _config |= static_cast<uint8_t>(channel << 5);   // bit6-5
        (void)writeConfig();
    }
    return true;
}

uint8_t MCP3424::getChannel()
{
    return _channel;
}

bool MCP3424::setGain(Gain gain)
{
    uint8_t g = static_cast<uint8_t>(gain);
    if ((g != 1) && (g != 2) && (g != 4) && (g != 8)) return false;
    if (_gain != g)
    {
        _gain = g;
        _config &= 0xFC;                       // 清增益位
        _config |= static_cast<uint8_t>(g - 1); // 1→00, 2→01, 4→10, 8→11
        (void)writeConfig();
    }
    return true;
}

uint8_t MCP3424::getGain()
{
    return _gain;
}

bool MCP3424::setResolution(Resolution bits)
{
    uint8_t b = static_cast<uint8_t>(bits);
    if ((b != 12) && (b != 14) && (b != 16) && (b != 18)) return false;
    if (_bits != b)
    {
        _bits = b;
        _config &= 0xF3;                       // 清分辨率位
        _config |= static_cast<uint8_t>(((b - 12) / 2) << 2);  // 12→00,14→01,16→10,18→11
        (void)writeConfig();
    }
    return true;
}

uint8_t MCP3424::getResolution()
{
    return _bits;
}

uint16_t MCP3424::getConversionDelay()
{
    // 典型转换时间表（ms）：12/14/16/18 bit
    static const uint16_t interval[4] = { 5, 17, 67, 267 };
    return interval[(_bits - 12) / 2];
}

void MCP3424::setContinuousMode()
{
    if (getMode() != 1)
    {
        _config |= 0x10;                       // bit4 = 1 连续
        (void)writeConfig();
    }
}

void MCP3424::setSingleShotMode()
{
    _config &= ~0x10;                          // bit4 = 0 单次
    _config |= 0x80;                           // RDY = 1 触发转换
    (void)writeConfig();
}

uint8_t MCP3424::getMode()
{
    return (_config & 0x10) ? 1 : 0;           // 1 = 连续，0 = 单次
}

// ============================================================
//  I2C 原语（无寄存器地址：直接写 1 字节 / 纯读 3~4 字节）
// ============================================================

bool MCP3424::writeConfig()
{
    _bus->lock();
    _bus->start();
    _bus->write_byte(_addr << 1);              // W
    bool ok = _bus->wait_ack() != 0;
    if (ok)
    {
        _bus->write_byte(_config);
        _bus->wait_ack();
    }
    _bus->stop();
    _bus->unlock();
    return ok;
}

int32_t MCP3424::readRaw()
{
    int32_t rv = 0;

    _bus->lock();
    _bus->start();
    _bus->write_byte((_addr << 1) | 0x01);     // R
    if (_bus->wait_ack() == 0)
    {
        _bus->stop();
        _bus->unlock();
        return 0;                              // 对齐原版：读失败返回 0
    }

    // 读数据字节 + 状态字节（18bit 为 4 字节，其余 3 字节）
    uint8_t n = (_bits == 18) ? 4 : 3;
    for (uint8_t i = 0; i < n; i++)
    {
        uint8_t b = _bus->read_byte();
        _bus->write_ack((i == (n - 1)) ? 1 : 0);   // 最后一字节 NACK
        if (i < (n - 1))
            rv = (rv << 8) | b;                // 数据在前，MSB first
        else
            _config = b;                       // 最后 1 字节 = 状态/配置
    }
    _bus->stop();
    _bus->unlock();

    rv = signExtend(rv, _bits);
    _raw = rv;
    return rv;
}

int32_t MCP3424::signExtend(int32_t rv, uint8_t bits) const
{
    // 按分辨率符号位扩展（对齐原版各分支）
    switch (bits)
    {
        case 12:
            if (rv & 0x0800) rv |= 0xFFFFF000;
            break;
        case 14:
            if (rv & 0x2000) rv |= 0xFFFFC000;
            break;
        case 16:
            if (rv & 0x8000) rv |= 0xFFFF0000;
            break;
        case 18:
            if (rv & 0x00020000) rv |= 0xFFFC0000;
            break;
        default:
            break;
    }
    return rv;
}
