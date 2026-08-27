#include "device_ds2401.hpp"
#include "systick.h"

// ============================================================
//  DS2401 1-Wire 唯一序列号芯片驱动
//  来源：Rob Tillaart 的 Arduino 库 "DS2401" v0.1.3
//    URL：https://github.com/RobTillaart/DS2401
//  移植说明见 device_ds2401.hpp 头注释
// ============================================================

namespace
{
// 1-Wire 标准速度时序常量（µs，见头注释时序表）
constexpr uint32_t RESET_LOW_US    = 480;   // 复位：拉低时长
constexpr uint32_t RESET_SAMPLE_US = 70;    // 释放后采样存在脉冲
constexpr uint32_t RESET_TOTAL_US  = 480;   // 释放后等待总时长
constexpr uint32_t SLOT_US         = 60;    // 时隙总长
constexpr uint32_t WRITE1_LOW_US   = 6;     // 写 1：拉低（1~15µs）
constexpr uint32_t READ_LOW_US     = 6;     // 读：拉低（1~15µs）
constexpr uint32_t READ_SAMPLE_US  = 9;     // 释放后采样（须 < 15µs）
constexpr uint32_t RECOVERY_US     = 2;     // 时隙间恢复
} // namespace

// ============================================================
//  构造 / 初始化
// ============================================================

DS2401::DS2401(io_ctrl& pin)
    : _pin(pin)
{
}

void DS2401::init()
{
    // 开漏输出 + 内部上拉：high() = 释放总线（读引脚），low() = 拉低
    _pin.init(mode_out_od, pullup, speed_medium);
    _pin.high();
}

// ============================================================
//  读序列号
// ============================================================

bool DS2401::readSerial(uint64_t& serial)
{
    if (!_reset())
    {
        return false;
    }
    _writeByte(CMD_READ_ROM);   // 0x33：仅单器件总线下有效

    uint8_t rom[ROM_LEN];
    for (uint8_t i = 0; i < ROM_LEN; i++)
    {
        rom[i] = _readByte();
    }

    if (!checkCRC(rom))
    {
        return false;
    }

    serial = 0;
    for (uint8_t i = 0; i < ROM_LEN; i++)
    {
        serial = (serial << 8) | rom[i];   // MSB 在前：家族码在最高字节
    }
    return true;
}

// ============================================================
//  CRC8（Dallas/Maxim：x8+x5+x4+1，LSB 先移）
// ============================================================

uint8_t DS2401::crc8(const uint8_t* data, uint8_t len)
{
    uint8_t crc = 0;
    while (len--)
    {
        crc ^= *data++;
        for (uint8_t i = 0; i < 8; i++)
        {
            crc = (crc & 0x01) ? (uint8_t)((crc >> 1) ^ 0x8C)
                               : (uint8_t)(crc >> 1);
        }
    }
    return crc;
}

bool DS2401::checkCRC(const uint8_t rom[ROM_LEN])
{
    return crc8(rom, ROM_LEN - 1) == rom[ROM_LEN - 1];
}

// ============================================================
//  1-Wire 底层时序（标准速度）
// ============================================================

bool DS2401::_reset()
{
    _pin.low();                               // 拉低 ≥ 480µs
    delay_us(RESET_LOW_US);
    _pin.high();                              // 释放（开漏 → 上拉）

    delay_us(RESET_SAMPLE_US);                // 从机 15~60µs 后拉低，持续 60~240µs
    bool presence = (_pin.read() == Low);   // 采样点仍在从机低电平窗口内

    delay_us(RESET_TOTAL_US - RESET_SAMPLE_US);  // 补满复位周期再操作
    return presence;
}

void DS2401::_writeBit(bool bit)
{
    _pin.low();
    if (bit)
    {
        // 写 1：短拉低后释放，剩余时隙由上拉维持高电平
        delay_us(WRITE1_LOW_US);
        _pin.high();
        delay_us(SLOT_US - WRITE1_LOW_US + RECOVERY_US);
    }
    else
    {
        // 写 0：全程拉低
        delay_us(SLOT_US);
        _pin.high();
        delay_us(RECOVERY_US);
    }
}

bool DS2401::_readBit()
{
    bool bit = false;
    _pin.low();
    delay_us(READ_LOW_US);
    _pin.high();                              // 释放，等待从机驱动

    delay_us(READ_SAMPLE_US);                 // 采样点须在 tRDV = 15µs 内
    bit = (_pin.read() == Hig);               // 从机拉低 = 逻辑 0

    delay_us(SLOT_US - READ_LOW_US - READ_SAMPLE_US + RECOVERY_US);
    return bit;
}

void DS2401::_writeByte(uint8_t byte)
{
    for (uint8_t i = 0; i < 8; i++)           // LSB 先发
    {
        _writeBit((byte & 0x01) != 0);
        byte >>= 1;
    }
}

uint8_t DS2401::_readByte()
{
    uint8_t byte = 0;
    for (uint8_t i = 0; i < 8; i++)           // LSB 先收
    {
        byte >>= 1;
        if (_readBit())
        {
            byte |= 0x80;
        }
    }
    return byte;
}
