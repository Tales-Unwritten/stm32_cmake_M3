#include "device_mcp_dac.hpp"
#include "systick.h"   // delay_us()

// ============================================================
//  MCP4921 / MCP4922 12 位 SPI DAC
//  来源：Rob Tillaart "MCP_DAC" v0.5.4
//    URL：https://github.com/RobTillaart/MCP_DAC
//  移植说明见 device_mcp_dac.hpp 头注释
// ============================================================

// ============================================================
//  构造 / 生命周期
// ============================================================

MCP4921::MCP4921(spi_bus &spi, uint8_t channels)
    : _spi(spi)
    , _latchPin(nullptr)
    , _channels((channels == 2) ? 2 : 1)
    , _gain(1)
    , _buffered(false)
    , _active(true)
    , _maxVoltage_mV(3300)          // 默认 Vref = 3.3 V
{
    reset();
}

void MCP4921::reset()
{
    // 增益默认 1x（输出最低，最安全）；数据手册 4.1.1.1 复位默认 2x
    _gain     = 1;
    _value[0] = 0;
    _value[1] = 0;
    _buffered = false;
    _active   = true;
}

// ============================================================
//  增益
// ============================================================

bool MCP4921::setGain(uint8_t gain)
{
    if ((gain == 0) || (gain > 2)) return false;
    _gain = gain;
    return true;
}

// ============================================================
//  输出
// ============================================================

bool MCP4921::write(uint16_t value, uint8_t channel)
{
    if (channel >= _channels) return false;

    // 钳位到 12 位（参考库存原值，此处存钳位值，见头注释）
    if (value > maxValue()) value = maxValue();
    _value[channel] = value;

    //  16 位帧：SHDN=1 正常工作；A/B、BUF、GA、12 位数据
    //    bit15 A/B（通道 B）| bit14 BUF | bit13 GA=1x | bit12 SHDN
    uint16_t data = 0x1000;                       // SHDN = 1
    if (channel == 1) data |= 0x8000;             // 通道 B
    if (_buffered)    data |= 0x4000;             // 输入缓冲
    if (_gain == 1)   data |= 0x2000;             // GA：1 = 1x，0 = 2x
    data |= value;

    transfer(data);
    return true;
}

uint16_t MCP4921::lastValue(uint8_t channel) const
{
    if (channel >= _channels) return 0;
    return _value[channel];
}

void MCP4921::fastWriteA(uint16_t value)
{
    // 固定配置：A 通道、BUF=0、GA=1x、SHDN=1
    transfer(0x3000u | (value & 0x0FFFu));
}

void MCP4921::fastWriteB(uint16_t value)
{
    // 固定配置：B 通道、BUF=0、GA=1x、SHDN=1
    transfer(0xB000u | (value & 0x0FFFu));
}

bool MCP4921::increment(uint8_t channel)
{
    if (channel >= _channels) return false;
    if (_value[channel] == maxValue()) return false;
    return write(static_cast<uint16_t>(_value[channel] + 1), channel);
}

bool MCP4921::decrement(uint8_t channel)
{
    if (channel >= _channels) return false;
    if (_value[channel] == 0) return false;
    return write(static_cast<uint16_t>(_value[channel] - 1), channel);
}

bool MCP4921::setPercentage_x100(uint16_t percentage, uint8_t channel)
{
    if (percentage > 10000) percentage = 10000;   // 钳位 0.00%..100.00%
    // 参考库：value = 0.01 × perc × maxValue → 定点：
    // value = perc_x100 × maxValue / 10000（int64 中间量）
    uint16_t value = static_cast<uint16_t>(
        static_cast<int64_t>(percentage) * maxValue() / 10000);
    return write(value, channel);
}

uint16_t MCP4921::getPercentage_x100(uint8_t channel) const
{
    if (channel >= _channels) return 0;
    return static_cast<uint16_t>(
        static_cast<int64_t>(_value[channel]) * 10000 / maxValue());
}

// ============================================================
//  电压换算（定点 mV）
// ============================================================

int32_t MCP4921::getVoltage_mV(uint8_t channel) const
{
    if (channel >= _channels) return 0;
    // mV = value × maxVoltage_mV / 4095（int64 中间量）
    return static_cast<int32_t>(
        static_cast<int64_t>(_value[channel]) * _maxVoltage_mV / maxValue());
}

// ============================================================
//  缓冲 / 关断 / LDAC
// ============================================================

void MCP4921::shutDown()
{
    _active = false;
    transfer(0x0000);   // SHDN=0 关断帧（输出高阻）；下一次 write 唤醒
}

void MCP4921::setLatchPin(io_ctrl *pin)
{
    _latchPin = pin;
    if (_latchPin != nullptr)
    {
        _latchPin->init(mode_out_pp, nopull, speed_medium);
        _latchPin->high();   // LDAC 空闲高电平
    }
}

void MCP4921::triggerLatch()
{
    if (_latchPin != nullptr)
    {
        _latchPin->low();
        delay_us(1);         // 数据手册 P7：LDAC 低电平 ≥ 100 ns
        _latchPin->high();
    }
}

// ============================================================
//  低层：16 位帧传输（先发高字节）
// ============================================================

void MCP4921::transfer(uint16_t data)
{
    _spi.cs_select();
    _spi.transfer_byte(static_cast<uint8_t>(data >> 8));
    _spi.transfer_byte(static_cast<uint8_t>(data & 0xFF));
    _spi.cs_deselect();
}
