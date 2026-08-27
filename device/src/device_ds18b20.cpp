//
//    FILE: device_ds18b20.cpp
//  AUTHOR: Rob Tillaart
// VERSION: 0.2.7（参考库 DS18B20_RT，移植自 Arduino）
//    DATE: 2017-07-25
// PURPOSE: DS18B20 单总线温度传感器驱动实现
//     URL: https://github.com/RobTillaart/DS18B20_RT
//
//  移植说明：
//   - 1-Wire 底层时序移植自 Paul Stoffregen 的 OneWire 库
//     （DS18B20_RT 的依赖），用 io_ctrl + delay_us 实现，
//     关键时隙内用 __disable_irq()/__enable_irq() 屏蔽中断
//     （对应参考库 noInterrupts()/interrupts()）
//   - 时隙边沿因 delay_us 校准误差与 reinit() 开销会有 ±1µs 平移，
//     ★ 真机需按实测调参 ★（见 device_ds18b20.hpp 头注释）

#include "device_ds18b20.hpp"
#include "inter_io_ctrl.hpp"
#include "delay.h"

namespace
{
// ── 1-Wire ROM 命令 ─────────────────────────────────────
constexpr uint8_t ONEWIRE_SEARCH_ROM = 0xF0;
constexpr uint8_t ONEWIRE_MATCH_ROM  = 0x55;
constexpr uint8_t ONEWIRE_SKIP_ROM   = 0xCC;

// ── DS18B20 功能命令 ────────────────────────────────────
constexpr uint8_t STARTCONVO   = 0x44;
constexpr uint8_t READSCRATCH  = 0xBE;
constexpr uint8_t WRITESCRATCH = 0x4E;

// ── 暂存器（ScratchPad）布局（偏移 0..8）─────────────────
//  [0] TEMP_LSB  [1] TEMP_MSB  [2] HIGH_ALARM  [3] LOW_ALARM
//  [4] CONFIG    [5] INTERNAL  [6] COUNT_REMAIN [7] COUNT_PER_C
//  [8] CRC
constexpr uint8_t TEMP_LSB       = 0;
constexpr uint8_t TEMP_MSB       = 1;
constexpr uint8_t SCRATCHPAD_CRC = 8;

// ── 分辨率配置字节（写入暂存器 CONFIGURATION）──────────
constexpr uint8_t TEMP_9_BIT  = 0x1F;   // 9 位，0.5°C
constexpr uint8_t TEMP_10_BIT = 0x3F;   // 10 位，0.25°C
constexpr uint8_t TEMP_11_BIT = 0x5F;   // 11 位，0.125°C
constexpr uint8_t TEMP_12_BIT = 0x7F;   // 12 位，0.0625°C

// ── 1-Wire 时序参数（µs，OneWire 库标准值）──────────────
//  ★ 真机需按实测调参 ★：delay_us 校准精度 + reinit() HAL 开销
//  （每次模式切换约 1µs）会平移时隙边沿，所有窗口仍在数据手册范围内
constexpr uint16_t OW_RESET_DELAY   = 480;   // 复位脉冲低电平
constexpr uint16_t OW_RESET_DELAY_2 = 70;    // 释放后等待存在脉冲
constexpr uint16_t OW_RESET_DELAY_3 = 410;   // 复位后总线恢复
constexpr uint16_t OW_WRITE_DELAY   = 60;    // 写 0 / 写 1 释放后保持
constexpr uint16_t OW_READ_DELAY    = 60;    // 读时隙释放后保持
constexpr uint16_t OW_WRITE_1_DELAY = 6;     // 写 1 低电平脉宽
constexpr uint16_t OW_READ_1_DELAY  = 6;     // 读时隙低电平脉宽
constexpr uint16_t OW_SAMPLE_DELAY  = 9;     // 释放后采样点
}   // namespace


// ────────────────────────────────────────────────────────
//  构造
// ────────────────────────────────────────────────────────

DeviceDS18B20::DeviceDS18B20(io_ctrl &onewire_pin, uint8_t resolution)
    : _pin(onewire_pin)
    , _addressFound(false)
    , _resolution(resolution)
    , _config(DS18B20_CLEAR)
    , _offset(0)
    , _lastDiscrepancy(0)
    , _lastFamilyDiscrepancy(0)
    , _lastDeviceFlag(false)
{
    // 总线引脚：开漏输出 + 内部上拉，空闲高电平
    if (_pin.is_initialized())
        _pin.reinit(mode_out_od, pullup, speed_low);
    else
        _pin.init(mode_out_od, pullup, speed_low);
    _pin.high();

    for (uint8_t i = 0; i < 8; i++)
    {
        _deviceAddress[i] = 0;
        _lastAddress[i]   = 0;
    }
}


// ────────────────────────────────────────────────────────
//  连接管理
// ────────────────────────────────────────────────────────

bool DeviceDS18B20::begin(uint8_t retries)
{
    _config = DS18B20_CLEAR;
    if (isConnected(retries))
    {
        _setResolution();
    }
    return _addressFound;
}


bool DeviceDS18B20::isConnected(uint8_t retries)
{
    _addressFound = false;
    for (uint8_t rtr = retries; (rtr > 0) && (_addressFound == false); rtr--)
    {
        _reset();
        _resetSearch();
        _deviceAddress[0] = 0x00;
        _search(_deviceAddress);
        _addressFound = (_deviceAddress[0] != 0x00) &&
                        (_crc8(_deviceAddress, 7) == _deviceAddress[7]);
    }
    return _addressFound;
}


bool DeviceDS18B20::getAddress(uint8_t *buf)
{
    if (_addressFound)
    {
        for (uint8_t i = 0; i < 8; i++)
        {
            buf[i] = _deviceAddress[i];
        }
    }
    return _addressFound;
}


// ────────────────────────────────────────────────────────
//  温度转换
// ────────────────────────────────────────────────────────

void DeviceDS18B20::requestTemperatures(void)
{
    _reset();
    _skip();
    _writeByte(STARTCONVO);
}


bool DeviceDS18B20::isConversionComplete(void)
{
    return (_readBit() == 1);
}


int32_t DeviceDS18B20::getTempCmC(bool checkConnect)
{
    uint8_t scratchPad[9];

    if (checkConnect)
    {
        if (isConnected(3) == false)
        {
            return DEVICE_DISCONNECTED;
        }
    }

    if (_config & DS18B20_CRC)
    {
        _readScratchPad(scratchPad, 9);
        if (_crc8(scratchPad, 8) != scratchPad[SCRATCHPAD_CRC])
        {
            return DEVICE_CRC_ERROR;
        }
        if (scratchPad[6] == 0x0C)
        {
            // 上电复位 85°C 假值（参考库 #37，需看 COUNT_REMAIN）
            if ((scratchPad[1] == 0x05) && (scratchPad[0] == 0x50))
            {
                return DEVICE_POR_ERROR;
            }
        }
    }
    else
    {
        _readScratchPad(scratchPad, 2);
    }

    // 127.94°C 假值 = 数据线对地短路
    if ((scratchPad[1] == 0x07) && (scratchPad[0] == 0xFF))
    {
        return DEVICE_GND_ERROR;
    }

    int16_t rawTemperature = (int16_t)(((int16_t)scratchPad[TEMP_MSB] << 8) |
                                        scratchPad[TEMP_LSB]);
    // 定点换算：0.0625°C/LSB = 62.5 m°C/LSB → raw × 125 / 2
    int32_t temp = ((int32_t)rawTemperature * 125) / 2;

    if (temp < DS18B20_MINIMUM)
    {
        return DEVICE_DISCONNECTED;
    }
    return temp + _offset;
}


// ────────────────────────────────────────────────────────
//  偏移 / 分辨率 / 配置
// ────────────────────────────────────────────────────────

void DeviceDS18B20::setOffset(int32_t offset)
{
    _offset = offset;
}


int32_t DeviceDS18B20::getOffset(void)
{
    return _offset;
}


bool DeviceDS18B20::setResolution(uint8_t resolution)
{
    if (isConnected())
    {
        _resolution = resolution;
        _setResolution();
    }
    return _addressFound;
}


uint8_t DeviceDS18B20::getResolution(void)
{
    return _resolution;
}


void DeviceDS18B20::setConfig(uint8_t config)
{
    _config = config;
}


uint8_t DeviceDS18B20::getConfig(void)
{
    return _config;
}


// ────────────────────────────────────────────────────────
//  1-Wire 底层
// ────────────────────────────────────────────────────────

void DeviceDS18B20::_busDriveLow(void)
{
    // 输出开漏 + 拉低（开漏保证总线释放后靠上拉恢复高电平）
    _pin.reinit(mode_out_od, pullup, speed_low);
    _pin.low();
}


void DeviceDS18B20::_busRelease(void)
{
    // 输入 + 内部上拉 = 释放总线
    _pin.reinit(mode_input, pullup);
}


bool DeviceDS18B20::_reset(void)
{
    uint8_t retries = 125;

    // 先等总线恢复高电平（最长 125 × 2µs）
    _busRelease();
    do
    {
        if (--retries == 0) return false;
        delay_us(2);
    } while (_pin.read() == Low);

    // 复位脉冲 480µs
    __disable_irq();
    _busDriveLow();
    __enable_irq();
    delay_us(OW_RESET_DELAY);

    // 释放 70µs 后采样存在脉冲（低 = 有器件）
    __disable_irq();
    _busRelease();
    delay_us(OW_RESET_DELAY_2);
    bool r = (_pin.read() == Low);
    __enable_irq();

    // 总线恢复期 410µs
    delay_us(OW_RESET_DELAY_3);
    return r;
}


void DeviceDS18B20::_writeBit(uint8_t v)
{
    if (v & 1)
    {
        // 写 1：拉低 6µs 释放，再保持 60µs（时隙总长 ~66µs）
        __disable_irq();
        _busDriveLow();
        delay_us(OW_WRITE_1_DELAY);
        _busRelease();
        __enable_irq();
        delay_us(OW_WRITE_DELAY);
    }
    else
    {
        // 写 0：拉低 60µs 释放，再保持 60µs
        __disable_irq();
        _busDriveLow();
        delay_us(OW_WRITE_DELAY);
        _busRelease();
        __enable_irq();
        delay_us(OW_WRITE_DELAY);
    }
}


uint8_t DeviceDS18B20::_readBit(void)
{
    // 读时隙：拉低 6µs → 释放 9µs → 采样 → 保持 60µs
    __disable_irq();
    _busDriveLow();
    delay_us(OW_READ_1_DELAY);
    _busRelease();
    delay_us(OW_SAMPLE_DELAY);
    uint8_t r = (_pin.read() == Hig) ? 1 : 0;
    __enable_irq();
    delay_us(OW_READ_DELAY);
    return r;
}


void DeviceDS18B20::_writeByte(uint8_t v)
{
    for (uint8_t bitMask = 0x01; bitMask != 0; bitMask <<= 1)
    {
        _writeBit((bitMask & v) ? 1 : 0);
    }
}


uint8_t DeviceDS18B20::_readByte(void)
{
    uint8_t r = 0;
    for (uint8_t bitMask = 0x01; bitMask != 0; bitMask <<= 1)
    {
        if (_readBit()) r |= bitMask;
    }
    return r;
}


void DeviceDS18B20::_select(const uint8_t *rom)
{
    _writeByte(ONEWIRE_MATCH_ROM);
    for (uint8_t i = 0; i < 8; i++)
    {
        _writeByte(rom[i]);
    }
}


void DeviceDS18B20::_skip(void)
{
    _writeByte(ONEWIRE_SKIP_ROM);
}


//  ROM 搜索（OneWire 库 search() 算法，支持总线多器件）
bool DeviceDS18B20::_search(uint8_t *newAddr)
{
    uint8_t id_bit_number;
    uint8_t last_zero;
    uint8_t rom_byte_number;
    uint8_t rom_byte_mask;
    uint8_t id_bit;
    uint8_t cmp_id_bit;
    uint8_t search_direction;
    bool    search_result = false;

    id_bit_number   = 1;
    last_zero       = 0;
    rom_byte_number = 0;
    rom_byte_mask   = 1;

    // 上一次搜索已到最后一个器件
    if (_lastDeviceFlag)
    {
        return false;
    }

    // 1-Wire 复位
    if (!_reset())
    {
        _lastDiscrepancy    = 0;
        _lastDeviceFlag     = false;
        _lastFamilyDiscrepancy = 0;
        return false;
    }

    // 发出搜索命令
    _writeByte(ONEWIRE_SEARCH_ROM);

    do
    {
        // 读一位及其反码
        id_bit      = _readBit();
        cmp_id_bit  = _readBit();

        // 总线无器件
        if ((id_bit == 1) && (cmp_id_bit == 1))
        {
            break;
        }

        if (id_bit != cmp_id_bit)
        {
            // 所有器件该位相同，按该位走
            search_direction = id_bit;
        }
        else
        {
            // 存在分叉：优先按上次搜索的路径走
            if (id_bit_number < _lastDiscrepancy)
            {
                search_direction =
                    (_lastAddress[rom_byte_number] & rom_byte_mask) ? 1 : 0;
            }
            else
            {
                // 等于最后一次分叉位置选 1，否则选 0
                search_direction = (id_bit_number == _lastDiscrepancy) ? 1 : 0;
            }

            // 选 0 时记录分叉位置
            if (search_direction == 0)
            {
                last_zero = id_bit_number;
                if (last_zero < _lastFamilyDiscrepancy)
                {
                    _lastFamilyDiscrepancy = last_zero;
                }
            }
        }

        // 写入本位的搜索方向
        if (search_direction == 1)
        {
            _lastAddress[rom_byte_number] |= rom_byte_mask;
        }
        else
        {
            _lastAddress[rom_byte_number] &= (uint8_t)~rom_byte_mask;
        }
        _writeBit(search_direction);

        // 推进位/字节计数
        id_bit_number++;
        rom_byte_mask <<= 1;
        if (rom_byte_mask == 0)
        {
            rom_byte_number++;
            rom_byte_mask = 1;
        }
    } while (rom_byte_number < 8);

    // 全部 64 位搜索完成
    if (!(id_bit_number < 65))
    {
        _lastDiscrepancy = last_zero;
        if (_lastDiscrepancy == 0)
        {
            _lastDeviceFlag = true;
        }
        search_result = true;
    }

    if (search_result)
    {
        for (uint8_t i = 0; i < 8; i++)
        {
            newAddr[i] = _lastAddress[i];
        }
    }
    return search_result;
}


void DeviceDS18B20::_resetSearch(void)
{
    _lastDiscrepancy    = 0;
    _lastDeviceFlag     = false;
    _lastFamilyDiscrepancy = 0;
    for (uint8_t i = 0; i < 8; i++)
    {
        _lastAddress[i] = 0;
    }
}


//  CRC-8（多项式 x^8 + x^5 + x^4 + 1，OneWire 库同款）
uint8_t DeviceDS18B20::_crc8(const uint8_t *data, uint8_t len)
{
    uint8_t crc = 0;
    while (len--)
    {
        uint8_t inbyte = *data++;
        for (uint8_t i = 8; i != 0; i--)
        {
            uint8_t mix = (crc ^ inbyte) & 0x01;
            crc >>= 1;
            if (mix) crc ^= 0x8C;
            inbyte >>= 1;
        }
    }
    return crc;
}


// ────────────────────────────────────────────────────────
//  私有：暂存器读写
// ────────────────────────────────────────────────────────

void DeviceDS18B20::_readScratchPad(uint8_t *scratchPad, uint8_t fields)
{
    _reset();
    _select(_deviceAddress);
    _writeByte(READSCRATCH);

    for (uint8_t i = 0; i < fields; i++)
    {
        scratchPad[i] = _readByte();
    }
    _reset();
}


void DeviceDS18B20::_setResolution(void)
{
    uint8_t res;
    switch (_resolution)
    {
        case 12: res = TEMP_12_BIT; break;
        case 11: res = TEMP_11_BIT; break;
        case 10: res = TEMP_10_BIT; break;
        // 最低分辨率兜底（参考库默认，只做整数运算）
        default: res = TEMP_9_BIT;  break;
    }

    _reset();
    _select(_deviceAddress);
    _writeByte(WRITESCRATCH);
    // 两个报警温度占位值
    _writeByte(0);
    _writeByte(100);
    _writeByte(res);
    _reset();
}


//  -- END OF FILE --
