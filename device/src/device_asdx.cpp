#include "device_asdx.hpp"
#include "systick.h"

// ============================================================
//  定点换算常量
// ============================================================

namespace
{
// 1 psi = 6894.75729 Pa → 定点 ×6894757/1000（误差 < 0.0002%）
constexpr int64_t PSI_TO_PA_NUM = 6894757;
constexpr int64_t PSI_TO_PA_DEN = 1000;

// 压力换算：14 位计数的 10%..90% → 0..maxPa
//   P = (raw - 1638) × maxPa / 13108
//   （原库系数 7.6289289e-5 = 1/13108，1638 = 0.10 × 16384）
constexpr int64_t ASDX_MIN_CNT = 1638;
constexpr int64_t ASDX_SPAN    = 13108;
}

// ============================================================
//  构造
// ============================================================

I2C_ASDX::I2C_ASDX(inter_i2c_bus* bus, uint8_t addr, uint8_t psi)
    : _dev(bus, addr)
    , _bus(bus)
    , _addr(addr)
    , _err(0)
    , _maxPa(0)
    , _pressure_Pa(0)
    , _rpc(0)
{
    // 量程 psi → Pa（仅合法值；原库同样只接受这 6 档）
    switch (psi)
    {
        case 1:   _maxPa = static_cast<int32_t>(1   * PSI_TO_PA_NUM / PSI_TO_PA_DEN); break;
        case 5:   _maxPa = static_cast<int32_t>(5   * PSI_TO_PA_NUM / PSI_TO_PA_DEN); break;
        case 15:  _maxPa = static_cast<int32_t>(15  * PSI_TO_PA_NUM / PSI_TO_PA_DEN); break;
        case 30:  _maxPa = static_cast<int32_t>(30  * PSI_TO_PA_NUM / PSI_TO_PA_DEN); break;
        case 60:  _maxPa = static_cast<int32_t>(60  * PSI_TO_PA_NUM / PSI_TO_PA_DEN); break;
        case 100: _maxPa = static_cast<int32_t>(100 * PSI_TO_PA_NUM / PSI_TO_PA_DEN); break;
        default:  _maxPa = 0; break;   // 非法量程 → 读数恒 0（对齐原库）
    }
}

void I2C_ASDX::init()
{
    (void)isConnected();
}

void I2C_ASDX::reset()
{
    _errorCount = 0;
    _lastRead = 0;
    _pressure_Pa = 0;
    _state = ERR_INIT;
}

// ============================================================
//  连接 / 错误
// ============================================================

bool I2C_ASDX::isConnected()
{
    _err = 0;
    bool ok = _dev.ping();
    if (!ok) _err = 1;
    return ok;
}

uint8_t I2C_ASDX::getLastError()
{
    uint8_t e = _err;
    _err = 0;
    return e;
}

// ============================================================
//  读取：ASDX 持续输出，直接读 2 字节
//   byte1: bit7-6 状态位 + bit5-0 压力高 6 位
//   byte2: 压力低 8 位
// ============================================================

int I2C_ASDX::read()
{
    _err = 0;

    _bus->lock();
    _bus->start();
    _bus->write_byte((_addr << 1) | 0x01);
    if (!_bus->wait_ack())
    {
        _err = 1;
        _bus->stop();
        _bus->unlock();
        _errorCount++;
        _state = ERR_READ;
        return _state;
    }

    uint16_t v = 0;
    for (uint8_t i = 0; i < 2; i++)
    {
        uint8_t b = _bus->read_byte();
        v = static_cast<uint16_t>((v << 8) | b);
        _bus->write_ack((i == 1) ? 1 : 0);
    }
    _bus->stop();
    _bus->unlock();

    _rpc = v;   // 14 位压力计数（bit13-0）

    // 状态位 0xC000 置位 = 传感器故障/未就绪（原库 C000_ERROR）
    if (_rpc & 0xC000)
    {
        _errorCount++;
        _state = ERR_STATUS;
        return _state;
    }

    // P = (raw - 1638) × maxPa / 13108（int64 防溢出）
    _pressure_Pa = static_cast<int32_t>(
        (static_cast<int64_t>(_rpc) - ASDX_MIN_CNT) * _maxPa / ASDX_SPAN);

    _lastRead = get_tick();
    _state = ERR_OK;
    return _state;
}

// ============================================================
//  单位换算
// ============================================================

int32_t I2C_ASDX::getPressure_psi() const
{
    // psi = Pa / 6894.75729 → ×100000/689475729（int64）
    return static_cast<int32_t>(
        (static_cast<int64_t>(_pressure_Pa) * 100000) / 689475729);
}

//  -- END OF FILE --
