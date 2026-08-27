#include "device_mprls.hpp"
#include "systick.h"

// ============================================================
//  传输函数 24 位计数边界（数据手册 / 原库常量）
// ============================================================

namespace
{
struct TfRange
{
    int64_t minCnt;   // 输出下限计数
    int64_t span;     // 输出计数跨度
};

TfRange tfRange(I2C_MPRLS::TransferFunction tf)
{
    switch (tf)
    {
        case I2C_MPRLS::TF_B:  return { 419430,   3355444  };   // 2.5%..22.5%
        case I2C_MPRLS::TF_C:  return { 3355444,  10066330 };   // 20%..80%
        case I2C_MPRLS::TF_A:
        default:               return { 1677722,  13421772 };   // 10%..90%
    }
}
}

// ============================================================
//  构造
// ============================================================

I2C_MPRLS::I2C_MPRLS(inter_i2c_bus* bus, uint8_t addr,
                     int32_t minPressure_Pa, int32_t maxPressure_Pa)
    : _dev(bus, addr)
    , _bus(bus)
    , _addr(addr)
    , _err(0)
    , _minPa(minPressure_Pa)
    , _maxPa(maxPressure_Pa)
{
    reset();
}

void I2C_MPRLS::init()
{
    if (!isConnected())
    {
        _error = ERR_CONNECT;
        return;
    }
    _error = ERR_OK;
}

void I2C_MPRLS::setPressureRange(int32_t minPressure_Pa, int32_t maxPressure_Pa)
{
    _minPa = minPressure_Pa;
    _maxPa = maxPressure_Pa;
}

void I2C_MPRLS::reset()
{
    _errorCount = 0;
    _lastRead = 0;
    _pressure_Pa = 0;
    _error = ERR_INIT;
    _state = 0x00;
}

// ============================================================
//  连接 / 错误
// ============================================================

bool I2C_MPRLS::isConnected()
{
    _err = 0;
    bool ok = _dev.ping();
    if (!ok) _err = 1;
    return ok;
}

uint8_t I2C_MPRLS::getLastError()
{
    uint8_t e = _err;
    _err = 0;
    return e;
}

// ============================================================
//  异步接口
// ============================================================

int I2C_MPRLS::request()
{
    // 测量命令帧：0xAA 0x00 0x00（freedom_write 的 reg 即首字节 0xAA）
    _dev.freedom_write(0xAA, 0x0000, 2);
    if (_dev.lastError() != 0)
    {
        _err = 1;
        _errorCount++;
        _error = ERR_WRITE;
        return _error;
    }
    _error = ERR_OK;
    return _error;
}

bool I2C_MPRLS::isBusy()
{
    uint64_t v = 0;
    if (!_readRaw(1, &v))
    {
        _errorCount++;
        _error = ERR_READ;
        return true;   // 读失败按"忙"处理，避免拿到过期数据
    }
    _state = static_cast<uint8_t>(v & 0xFF);
    return (_state & STATUS_BUSY) != 0;
}

bool I2C_MPRLS::conversionReady()
{
    return !isBusy();
}

int I2C_MPRLS::getData()
{
    uint64_t v = 0;
    if (!_readRaw(4, &v))
    {
        _errorCount++;
        _error = ERR_READ;
        return _error;
    }

    // 状态 + 24 位压力（MSB 优先）
    _state = static_cast<uint8_t>((v >> 24) & 0xFF);
    _rpc   = static_cast<int32_t>(v & 0xFFFFFF);

    // 压力 Pa：P = (raw - minCnt) × (maxP - minP) / span + minP
    TfRange r = tfRange(_transferFunction);
    _pressure_Pa = static_cast<int32_t>(
        (static_cast<int64_t>(_rpc) - r.minCnt) *
            (_maxPa - _minPa) / r.span + _minPa);

    _lastRead = get_tick();
    _error = ERR_OK;
    return _error;
}

// ============================================================
//  阻塞接口
// ============================================================

int I2C_MPRLS::read()
{
    // 新的 read() 使上次状态失效
    _state = 0x00;

    if (request() != ERR_OK) return _error;
    delay_ms(5);              // 数据手册保守转换时间
    return getData();
}

// ============================================================
//  纯读（无寄存器指针；inter_i2c_dev 不支持，直接操作总线）
// ============================================================

bool I2C_MPRLS::_readRaw(uint8_t len, uint64_t* out)
{
    if (!out || len == 0 || len > 4) return false;
    *out = 0;
    _err = 0;

    _bus->lock();
    _bus->start();
    _bus->write_byte((_addr << 1) | 0x01);
    if (!_bus->wait_ack())
    {
        _err = 1;
        _bus->stop();
        _bus->unlock();
        return false;
    }

    for (uint8_t i = 0; i < len; i++)
    {
        uint8_t b = _bus->read_byte();
        *out = (*out << 8) | b;
        _bus->write_ack((i == len - 1) ? 1 : 0);   // 最后一字节 NACK
    }
    _bus->stop();
    _bus->unlock();
    return true;
}

//  -- END OF FILE --
