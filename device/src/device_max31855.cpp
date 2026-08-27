#include "device_max31855.hpp"
#include "systick.h"   // get_tick()

// ============================================================
//  MAX31855 K 型热电偶温度计（SPI）
//  来源：Rob Tillaart "MAX31855_RT" v0.6.2
//    URL：https://github.com/RobTillaart/MAX31855_RT
//  移植说明见 device_max31855.hpp 头注释
// ============================================================

// ============================================================
//  构造 / 生命周期
// ============================================================

MAX31855::MAX31855(spi_bus &spi)
    : _spi(spi)
    , _status(STATUS_NOREAD)
    , _internal_mC(NO_TEMPERATURE_mC)
    , _temperature_mC(NO_TEMPERATURE_mC)
    , _offset_mC(0)
    , _seebeck_x1000(K_TC)
    , _lastTimeRead(0)
    , _rawData(0)
{
}

void MAX31855::begin()
{
    _status          = STATUS_NOREAD;
    _internal_mC     = NO_TEMPERATURE_mC;
    _temperature_mC  = NO_TEMPERATURE_mC;
    _offset_mC       = 0;
    _seebeck_x1000   = K_TC;
    _lastTimeRead    = 0;
    _rawData         = 0;
}

// ============================================================
//  读取
// ============================================================

uint32_t MAX31855::_read()
{
    _rawData = 0;
    _spi.cs_select();
    for (uint8_t i = 0; i < 4; i++)
    {
        _rawData = (_rawData << 8) | _spi.transfer_byte(0);
    }
    _spi.cs_deselect();
    return _rawData;
}

uint8_t MAX31855::read()
{
    //  32 位帧位序（数据手册 P10）：
    //    bit31         温度符号位
    //    bit30..18     热电偶温度，13 位幅值，0.25°C/LSB
    //    bit17         保留（恒 0）
    //    bit16         故障位（与 bit2..0 状态位冗余）
    //    bit15..4      冷端温度，12 位（bit15 符号），0.0625°C/LSB
    //    bit3          保留（恒 0）
    //    bit2..0       状态：bit2 SCV / bit1 SCG / bit0 OC
    uint32_t value = _read();

    //  全 1 帧 = MISO 无上拉或未连接（bit3/bit17 恒 0，P10 数据手册）
    if (value == 0xFFFFFFFFu)
    {
        _status = STATUS_NO_COMMUNICATION;
        return _status;
    }

    _lastTimeRead = get_tick();

    // ── 状态位 bit2..0 ──
    _status = static_cast<uint8_t>(value & 0x0007u);
    // 故障时温度仍有效（参考库 0.4.0 起不再提前返回）

    // ── 冷端温度 bit15..4（12 位补码，0.0625°C/LSB）──
    // 定点：0.0625°C = 62.5 m°C = 125/2 m°C
    int32_t internal = static_cast<int32_t>((value >> 4) & 0x0FFFu);
    if (internal & 0x0800) internal -= 0x1000;          // 符号扩展
    _internal_mC = (internal * 125) / 2;                // 截断误差 ≤ 0.5 m°C

    // ── 热电偶温度 bit31..18（14 位补码，0.25°C/LSB）──
    // 定点：0.25°C = 250 m°C，精确无误差
    int32_t temp = static_cast<int32_t>((value >> 18) & 0x3FFFu);
    if (temp & 0x2000) temp -= 0x4000;                  // 符号扩展
    _temperature_mC = temp * 250;

    return _status;
}

int32_t MAX31855::getTemperature_mC()
{
    // 默认 K 型：直读 + 偏移（偏移在乘系数之后加，参考库行为）
    if (_seebeck_x1000 == K_TC)
    {
        return _temperature_mC + _offset_mC;
    }

    // 其他热电偶（实验性，参考库同款近似，线性修正）：
    //   1) 由 K 型读数反推热端电压：Vout[µV] = K_TC[µV/°C] × ΔT[°C]
    //      ΔT_m = 温度 - 冷端（m°C），ΔT = ΔT_m/1000
    //      Vout = 41.276 × ΔT_m/1000 = 41276 × ΔT_m / 1e6
    //   2) 用目标 Seebeck 系数还原温度：
    //      T'[m°C] = Vout/SC[°C] × 1000 = Vout × 1e6 / SC_x1000
    int64_t dT_mC = static_cast<int64_t>(_temperature_mC) - _internal_mC;
    int64_t vout_uV = (static_cast<int64_t>(K_TC) * dT_mC) / 1000000LL;
    int64_t t_mC = (vout_uV * 1000000LL) / _seebeck_x1000
                 + _internal_mC + _offset_mC;
    return static_cast<int32_t>(t_mC);
}
