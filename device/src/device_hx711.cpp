//
//    FILE: device_hx711.cpp
//  AUTHOR: Rob Tillaart
// VERSION: 0.6.4（参考库 HX711，移植自 Arduino）
//    DATE: 2019-09-04
// PURPOSE: HX711 24 位 ADC 称重传感器驱动实现
//     URL: https://github.com/RobTillaart/HX711
//
//  移植说明：
//   - 时序 µs 级：PD_SCK 每半周期 1µs（参考库 fastProcessor 分支，
//     T2 ≥ 0.2µs）；掉电 60µs → 取 64µs；均从数据手册原样移植

#include "device_hx711.hpp"
#include "systick.h"   // delay_us() / delay_ms()

namespace
{
// 掉电 PD_SCK 高电平时间（µs，数据手册 ≥ 60µs，参考库取 64µs）
constexpr uint32_t HX711_POWER_DOWN_US = 64;
// 时钟半周期（µs，参考库 fastProcessor 分支；T2 ≥ 0.2µs）
constexpr uint32_t HX711_CLK_HALF_US = 1;
}   // namespace


// ────────────────────────────────────────────────────────
//  构造 / 初始化 / 复位
// ────────────────────────────────────────────────────────

DeviceHX711::DeviceHX711(io_ctrl &dataPin, io_ctrl &clockPin)
    : _data(dataPin)
    , _clock(clockPin)
    , _offset(0)
    , _gain(CHANNEL_A_GAIN_128)
{
    // DOUT：输入 + 上拉（参考库 INPUT_PULLUP）
    if (_data.is_initialized())
        _data.reinit(mode_input, pullup);
    else
        _data.init(mode_input, pullup);

    // PD_SCK：推挽输出，空闲低电平（参考库 digitalWrite LOW）
    if (_clock.is_initialized())
        _clock.reinit(mode_out_pp, nopull, speed_low);
    else
        _clock.init(mode_out_pp, nopull, speed_low);
    _clock.low();
}


void DeviceHX711::begin(bool doReset)
{
    if (doReset)
    {
        reset();
    }
}


void DeviceHX711::reset()
{
    powerDown();
    powerUp();
    _offset = 0;
    _gain   = CHANNEL_A_GAIN_128;
    read();   // 强制设置增益（参考库 reset 同款）
}


// ────────────────────────────────────────────────────────
//  就绪查询
// ────────────────────────────────────────────────────────

bool DeviceHX711::isReady()
{
    return _data.read() == Low;   // DOUT 低 = 数据就绪
}


void DeviceHX711::waitReady(uint32_t ms)
{
    while (!isReady())
    {
        delay_ms(ms);
    }
}


bool DeviceHX711::waitReadyRetry(uint8_t retries, uint32_t ms)
{
    while (retries--)
    {
        if (isReady()) return true;
        delay_ms(ms);
    }
    return false;
}


// ────────────────────────────────────────────────────────
//  读取
// ────────────────────────────────────────────────────────

//  数据手册 Page 4：DOUT 高 = 数据未就绪；PD_SCK 空闲必须为低。
//  阻塞等待期中断保持开启；24 位移位 + 增益脉冲期间关中断
//  （参考库 noInterrupts() 同款，窗口约 60~120µs）
int32_t DeviceHX711::read()
{
    // 阻塞等待数据就绪；首次读取最长 ~400ms（数据手册 Page 3）
    while (_data.read() == Hig)
    {
    }

    uint8_t data[3];

    __disable_irq();

    // 24 个脉冲读 24 位（MSB 先入）
    data[2] = _shiftIn();
    data[1] = _shiftIn();
    data[0] = _shiftIn();

    // 第 25 / 26 / 27 个脉冲选择下一轮转换的通道/增益
    // （数据手册 Table 3）
    uint8_t m = 1;
    if      (_gain == CHANNEL_A_GAIN_64) m = 3;
    else if (_gain == CHANNEL_B_GAIN_32) m = 2;

    while (m > 0)
    {
        _clock.high();
        delay_us(HX711_CLK_HALF_US);   // T2 ≥ 0.2µs
        _clock.low();
        delay_us(HX711_CLK_HALF_US);   // 保持 ~50% 占空比
        m--;
    }

    __enable_irq();

    // 24 位符号扩展（参考库 union + 符号位检查同义）
    int32_t value = (int32_t)(((uint32_t)data[2] << 16) |
                              ((uint32_t)data[1] << 8) |
                              data[0]);
    if (data[2] & 0x80)
    {
        value -= 0x1000000;   // 减 2^24 = 补码符号扩展
    }

    return value;
}


int32_t DeviceHX711::readAverage(uint8_t times)
{
    if (times < 1) times = 1;
    int64_t sum = 0;   // 24 位值 × 最多 255 次可能超过 int32，用 int64 累加
    for (uint8_t i = 0; i < times; i++)
    {
        sum += read();
    }
    return (int32_t)(sum / times);
}


int32_t DeviceHX711::getValue(uint8_t times)
{
    return readAverage(times) - _offset;
}


// ────────────────────────────────────────────────────────
//  去皮
// ────────────────────────────────────────────────────────

void DeviceHX711::tare(uint8_t times)
{
    _offset = readAverage(times);
}


//  参考库 get_tare() = -offset × scale；本移植无 scale（恒为 1）
int32_t DeviceHX711::getTare()
{
    return -_offset;
}


bool DeviceHX711::tareSet()
{
    return _offset != 0;
}


// ────────────────────────────────────────────────────────
//  增益 / 通道
// ────────────────────────────────────────────────────────

bool DeviceHX711::setGain(uint8_t gain, bool forced)
{
    if (!forced && (_gain == gain)) return true;

    switch (gain)
    {
        case CHANNEL_B_GAIN_32:
        case CHANNEL_A_GAIN_64:
        case CHANNEL_A_GAIN_128:
            _gain = gain;
            read();   // 下一次用户 read() 从正确的通道/增益开始
            return true;
    }
    return false;   // 非法值，不改变
}


uint8_t DeviceHX711::getGain()
{
    return _gain;
}


// ────────────────────────────────────────────────────────
//  零偏
// ────────────────────────────────────────────────────────

void DeviceHX711::setOffset(int32_t offset)
{
    _offset = offset;
}


int32_t DeviceHX711::getOffset()
{
    return _offset;
}


// ────────────────────────────────────────────────────────
//  电源管理
// ────────────────────────────────────────────────────────

void DeviceHX711::powerDown()
{
    // PD_SCK 高电平 ≥ 60µs 进入掉电（数据手册）
    _clock.high();
    delay_us(HX711_POWER_DOWN_US);
}


void DeviceHX711::powerUp()
{
    _clock.low();
}


// ────────────────────────────────────────────────────────
//  私有
// ────────────────────────────────────────────────────────

//  MSB 先入移位（参考库 _shiftIn fastProcessor 版）
uint8_t DeviceHX711::_shiftIn()
{
    uint8_t value = 0;
    for (uint8_t mask = 0x80; mask != 0; mask >>= 1)
    {
        _clock.high();
        delay_us(HX711_CLK_HALF_US);   // T2 ≥ 0.2µs，上升沿后数据稳定
        if (_data.read() == Hig)
        {
            value |= mask;
        }
        _clock.low();
        delay_us(HX711_CLK_HALF_US);   // 保持 ~50% 占空比
    }
    return value;
}


//  -- END OF FILE --
