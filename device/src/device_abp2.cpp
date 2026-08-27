#include "device_abp2.hpp"
#include "systick.h"

// ============================================================
//  定点换算常量（数据手册：输出为 2^24 计数的 10%..90%）
// ============================================================

namespace
{
// 24 位计数的 10% / 90% 边界
constexpr int64_t ABP2_MIN_CNT = 1677722;     // 0.10 × 2^24
constexpr int64_t ABP2_MAX_CNT = 15099494;    // 0.90 × 2^24
constexpr int64_t ABP2_SPAN    = ABP2_MAX_CNT - ABP2_MIN_CNT;   // 13421772

// 温度：T(°C) = raw × 200/16777215 - 50 → m°C = raw×200000/16777215 - 50000
constexpr int64_t TEMP_RAW_MAX = 16777215;
}

// ============================================================
//  构造
// ============================================================

I2C_ABP2::I2C_ABP2(inter_i2c_bus* bus, uint8_t addr,
                   int32_t minPressure_Pa, int32_t maxPressure_Pa)
    : _dev(bus, addr)
    , _bus(bus)
    , _addr(addr)
    , _err(0)
    , _minPa(minPressure_Pa)
    , _maxPa(maxPressure_Pa)
{
}

void I2C_ABP2::init()
{
    (void)isConnected();
}

void I2C_ABP2::setPressureRange(int32_t minPressure_Pa, int32_t maxPressure_Pa)
{
    _minPa = minPressure_Pa;
    _maxPa = maxPressure_Pa;
}

// ============================================================
//  连接 / 错误
// ============================================================

bool I2C_ABP2::isConnected()
{
    _err = 0;
    bool ok = _dev.ping();
    if (!ok) _err = 1;
    return ok;
}

uint8_t I2C_ABP2::getLastError()
{
    uint8_t e = _err;
    _err = 0;
    return e;
}

// ============================================================
//  异步接口
// ============================================================

int I2C_ABP2::request()
{
    // 命令帧：0xAA 0x00 0x00（freedom_write 的 reg 即首字节 0xAA）
    _dev.freedom_write(0xAA, 0x0000, 2);
    if (_dev.lastError() != 0)
    {
        _err = 1;
        return ERR_REQUEST;
    }
    _err = 0;
    return ERR_OK;
}

int I2C_ABP2::read()
{
    // 纯读 7 字节：状态(1) + 压力(3) + 温度(3)，无寄存器指针
    // （数据手册 6.8：命令帧后直接读）
    _err = 0;

    _bus->lock();
    _bus->start();
    _bus->write_byte((_addr << 1) | 0x01);
    if (!_bus->wait_ack())
    {
        _err = 1;
        _bus->stop();
        _bus->unlock();
        return ERR_READ;
    }

    uint64_t v = 0;
    for (uint8_t i = 0; i < 7; i++)
    {
        uint8_t b = _bus->read_byte();
        v = (v << 8) | b;
        _bus->write_ack((i == 6) ? 1 : 0);   // 最后一字节 NACK
    }
    _bus->stop();
    _bus->unlock();

    _state = static_cast<uint8_t>((v >> 48) & 0xFF);
    _rawP  = static_cast<uint32_t>((v >> 24) & 0xFFFFFF);
    _rawT  = static_cast<uint32_t>(v & 0xFFFFFF);

    // 压力 Pa：线性映射 10%..90% 计数 → minPa..maxPa
    //   P = (raw - MIN) × (maxP - minP) / SPAN + minP
    _pressure_Pa = static_cast<int32_t>(
        (static_cast<int64_t>(_rawP) - ABP2_MIN_CNT) *
            (_maxPa - _minPa) / ABP2_SPAN + _minPa);

    // 温度 m°C
    _temperature_mC = static_cast<int32_t>(
        static_cast<int64_t>(_rawT) * 200000 / TEMP_RAW_MAX - 50000);

    _lastRead = get_tick();
    return ERR_OK;
}

//  -- END OF FILE --
