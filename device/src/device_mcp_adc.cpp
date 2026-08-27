#include "device_mcp_adc.hpp"

// ============================================================
//  MCP3204 / MCP3208 12 位 SPI ADC
//  来源：Rob Tillaart "MCP_ADC" v0.5.2
//    URL：https://github.com/RobTillaart/MCP_ADC
//  移植说明见 device_mcp_adc.hpp 头注释
// ============================================================

// ============================================================
//  构造 / 生命周期
// ============================================================

MCP3208::MCP3208(spi_bus &spi, uint8_t channels)
    : _spi(spi)
    , _channels((channels == 4) ? 4 : 8)
    , _vref_uV(3300ULL * 1000)      // 默认 Vref = 3.3 V
    , _count(0)
{
}

void MCP3208::init()
{
    // 参考库 begin()：CS 脉冲强制通信（上电后首次转换可能损坏）。
    // spi_port 的 CS 若未配置则由用户在外部控制，此哑读等效于脉冲。
    (void)read(0);
}

// ============================================================
//  低层：命令帧 + 3 字节全双工
// ============================================================

int16_t MCP3208::readADC(uint8_t channel, bool single)
{
    if (channel >= _channels) return 0;
    _count++;

    //  命令帧（数据手册 fig 6.1，MCP3204/3208）：
    //    byte0: 0x04 起始位 | 0x02 单端 | 0x01 D2（通道 bit2）
    //    byte1: D1 D0（通道 bit1..0）置于 bit7..6
    //  参考库写作 data[1] |= (channel << 6)，对 8 位截断后与
    //  (channel & 0x03) << 6 等价，此处直接写等价形式
    uint8_t data[3] = { 0, 0, 0 };
    data[0] = 0x04u | (single ? 0x02u : 0x00u) | ((channel > 3) ? 0x01u : 0x00u);
    data[1] = static_cast<uint8_t>((channel & 0x03u) << 6);

    _spi.cs_select();
    for (uint8_t b = 0; b < 3; b++)
    {
        data[b] = _spi.transfer_byte(data[b]);
    }
    _spi.cs_deselect();

    //  应答：byte1 低 4 位 + byte2 全部 = 12 位数据
    uint16_t raw = (static_cast<uint16_t>(data[1]) << 8) | data[2];
    return static_cast<int16_t>(raw & 0x0FFFu);
}

// ============================================================
//  测量 API
// ============================================================

int16_t MCP3208::read(uint8_t channel)
{
    if (channel >= _channels) return 0;
    return readADC(channel, true);
}

int16_t MCP3208::differentialRead(uint8_t channel)
{
    if (channel >= _channels) return 0;
    return readADC(channel, false);
}

int16_t MCP3208::deltaRead(uint8_t channel)
{
    if (channel >= _channels) return 0;

    // 参考库逻辑原样移植：
    //  读差分对 (ch & ~1) 和 (ch | 1) 并求差；
    //  val0 为正时省去第二次读取（12 位原始值按无符号判断）。
    int16_t val0 = differentialRead(channel & 0xFEu);
    int16_t val1 = (val0 > 0) ? 0 : differentialRead(channel | 0x01u);

    if (channel & 0x01u) return static_cast<int16_t>(val1 - val0);
    return static_cast<int16_t>(val0 - val1);
}

void MCP3208::readMultiple(const uint8_t channels[], uint8_t numChannels, int16_t readings[])
{
    for (uint8_t i = 0; i < numChannels; i++)
    {
        readings[i] = readADC(channels[i], true);
    }
}

// ============================================================
//  电压换算（定点 uV）
// ============================================================

int32_t MCP3208::rawToVoltage_uV(int16_t raw) const
{
    //  uV = raw × Vref_uV / 4096
    //  raw ≤ 4095，Vref_uV ≤ ~33e6（Vref ≤ 33 V）→ 乘积 ≤ 1.4e11，int64 安全
    int64_t v = static_cast<int64_t>(raw & 0x0FFFu) * static_cast<int64_t>(_vref_uV);
    return static_cast<int32_t>(v / 4096);
}

int32_t MCP3208::read_uV(uint8_t channel)
{
    return rawToVoltage_uV(read(channel));
}

uint32_t MCP3208::count()
{
    uint32_t cnt = _count;
    _count = 0;
    return cnt;
}
