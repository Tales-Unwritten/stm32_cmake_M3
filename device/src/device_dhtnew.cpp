//
//    FILE: device_dhtnew.cpp
//  AUTHOR: Rob Tillaart
// VERSION: 0.5.5（参考库 DHTNEW，移植自 Arduino）
//    DATE: 2023-12-30
// PURPOSE: DHT11 / DHT22 温湿度传感器驱动实现
//     URL: https://github.com/RobTillaart/DHTNEW
//
//  移植说明：
//   - 时序参数从参考库原样移植，本移植无 micros()，
//     超时与位宽用 delay_us(1) 轮询计数实现
//     ★ 真机需按实测调参 ★：delay_us(1) 实测时长偏离 1µs 时，
//     需等比修正 DHTLIB_BIT_THRESHOLD 与各超时值（见头文件）

#include "device_dhtnew.hpp"
#include "systick.h"

namespace
{
// ── 唤醒拉低时间（µs，参考库加 10% 余量）────────────────
constexpr uint32_t DHTLIB_DHT11_WAKEUP = 19800;   // 18ms × 1.1
constexpr uint32_t DHTLIB_DHT_WAKEUP   = 1100;    // 1ms × 1.1

// ── 位采样超时（µs，参考库原值）────────────────────────
//  ★ 真机需按实测调参 ★（轮询计数与 delay_us(1) 精度相关）
constexpr uint32_t WAITFORSENSOR = 50;            // DHT22 响应等待
constexpr uint32_t WAITFORSENSOR_DHT11 = 15000;   // DHT11 响应等待（6~10ms 拉低）
constexpr uint32_t TIMEOUT_BIT = 90;              // A/B/C/D 位超时
}   // namespace


// ────────────────────────────────────────────────────────
//  构造 / 复位
// ────────────────────────────────────────────────────────

DeviceDHTNEW::DeviceDHTNEW(io_ctrl &data_pin)
    : _pin(data_pin)
    , _type(0)
    , _wakeupDelay(0)
    , _humidity10(0)
    , _temperaturemC(0)
    , _humidityOffset(0)
    , _temperatureOffset(0)
    , _disableIRQ(true)
    , _suppressError(false)
{
    for (uint8_t i = 0; i < 5; i++) _bits[i] = 0;
    _pinOut();
    _pin.high();   // 总线空闲高电平
}


void DeviceDHTNEW::reset()
{
    // 数据总线空闲状态为高电平（参考库 reset()）
    _pinOut();
    _pin.high();

    _wakeupDelay        = 0;
    _type               = 0;
    _humidityOffset     = 0;
    _temperatureOffset  = 0;
    _humidity10         = 0;
    _temperaturemC      = 0;
    _disableIRQ         = true;
    _suppressError      = false;
    for (uint8_t i = 0; i < 5; i++) _bits[i] = 0;
}


// ────────────────────────────────────────────────────────
//  类型
// ────────────────────────────────────────────────────────

uint8_t DeviceDHTNEW::getType()
{
    if (_type == 0) read();
    return _type;
}


void DeviceDHTNEW::setType(uint8_t type)
{
    _type = 0;
    _wakeupDelay = DHTLIB_DHT11_WAKEUP;

    if ((type == 22) || (type == 23))
    {
        _type = 22;   // 23 映射为 22（参考库：无法区分）
        _wakeupDelay = DHTLIB_DHT_WAKEUP;
    }
    else if (type == 11)
    {
        _type = type;
        _wakeupDelay = DHTLIB_DHT11_WAKEUP;
    }
}


// ────────────────────────────────────────────────────────
//  核心读取（含自动识别）
// ────────────────────────────────────────────────────────

int8_t DeviceDHTNEW::read()
{
    if (_type != 0)
    {
        return _read();
    }

    // ── 自动识别 ──
    // 先按 DHT22 读；DHT22 编码下湿度最大 100.0% = 0x03E8，
    // 高位字节不可能 > 0x03，超出则说明是 KY015/DHT11 编码（参考库 #102/#104）
    _type = 22;
    _wakeupDelay = DHTLIB_DHT_WAKEUP;
    int8_t rv = _read();
    if (rv == DHTLIB_OK)
    {
        if (_bits[0] <= 0x03)
        {
            return rv;
        }
        _type = 11;
        _wakeupDelay = DHTLIB_DHT11_WAKEUP;
        rv = _read();   // 按 DHT11 换算重读
        return rv;
    }

    _type = 11;
    _wakeupDelay = DHTLIB_DHT11_WAKEUP;
    rv = _read();
    if (rv == DHTLIB_OK)
    {
        return rv;
    }

    _type = 0;   // 识别失败，下次再试
    return rv;
}


//  读 40 位 → 定点换算 → 校验
int8_t DeviceDHTNEW::_read()
{
    int8_t rv = _readSensor();

    // 总线恢复空闲高电平（参考库在读取结束后做）
    _pinOut();
    _pin.high();

    if (rv != DHTLIB_OK)
    {
        if (_suppressError == false)
        {
            _humidity10    = DHTLIB_INVALID_HUMIDITY;
            _temperaturemC = DHTLIB_INVALID_TEMPERATURE;
        }
        return rv;
    }

    // ── 定点换算 ──
    if (_type == 11)   // DHT11 / KY015 兼容
    {
        _humidity10    = _bits[0] * 10 + _bits[1];           // 0.1%RH
        _temperaturemC = _bits[2] * 1000 + _bits[3] * 100;   // m°C
    }
    else               // DHT22 兼容（16 位 ×10 定点）
    {
        _humidity10 = (_bits[0] << 8) | _bits[1];            // 0.1%RH

        // 正温度
        if ((_bits[2] & 0x80) == 0x00)
        {
            int16_t t = (int16_t)(((_bits[2] & 0x7F) << 8) | _bits[3]);
            _temperaturemC = t * 100;
        }
        else   // 负温度：两种表示（参考库 #100）
        {
            if ((_bits[2] & 0x40) == 0x00)
            {
                // 符号-幅值：0x80 为符号位
                int16_t t = (int16_t)(((_bits[2] & 0x7F) << 8) | _bits[3]);
                _temperaturemC = -t * 100;
            }
            else
            {
                // 16 位补码
                int16_t t = (int16_t)((_bits[2] << 8) | _bits[3]);
                _temperaturemC = t * 100;
            }
        }
    }

    // ── 偏移（参考库顺序：先加偏移再校验和）────────────
    if (_humidityOffset != 0)
    {
        _humidity10 += _humidityOffset;
        if (_humidity10 > 1000)      _humidity10 = 1000;   // 限幅 0..100.0%
        else if (_humidity10 < 0)    _humidity10 = 0;
    }
    if (_temperatureOffset != 0)
    {
        _temperaturemC += _temperatureOffset;
    }

    // ── 校验和 ──
    uint8_t sum = _bits[0] + _bits[1] + _bits[2] + _bits[3];
    if (_bits[4] != sum)
    {
        return DHTLIB_ERROR_CHECKSUM;
    }

    return DHTLIB_OK;
}


// ────────────────────────────────────────────────────────
//  私有：唤醒 + 响应检测 + 40 位采样
// ────────────────────────────────────────────────────────

int8_t DeviceDHTNEW::_readSensor()
{
    uint8_t mask = 0x80;
    uint8_t idx  = 0;

    for (uint8_t i = 0; i < 5; i++) _bits[i] = 0;

    // ── 唤醒：主机拉低总线 ──
    _pinOut();
    _pin.low();
    delay_us(_wakeupDelay);   // DHT11 19800µs / DHT22 1100µs

    // ── 释放总线，等待传感器响应 ──
    _pin.high();
    delay_us(2);
    _pinIn();

    // 响应检测（非时序关键，保持中断开启，
    // 避免 DHT11 最长 ~15ms 的关中断时间——参考库全程关中断）
    uint32_t waitForSensor = WAITFORSENSOR;
    if (_type == 11) waitForSensor = WAITFORSENSOR_DHT11;
    if (_waitFor(0, waitForSensor)) return DHTLIB_ERROR_SENSOR_NOT_READY;
    // 传感器拉低 ~80µs 后拉高 ~80µs（应答信号）
    if (_waitFor(1, TIMEOUT_BIT)) return DHTLIB_ERROR_TIMEOUT_A;
    if (_waitFor(0, TIMEOUT_BIT)) return DHTLIB_ERROR_TIMEOUT_B;

    // ── 40 位数据采样（时序关键，关中断防位宽抖动）──
    if (_disableIRQ) __disable_irq();

    for (uint8_t i = 40; i != 0; i--)
    {
        // 每位以 ~50µs 低电平开始
        if (_waitFor(1, TIMEOUT_BIT))
        {
            if (_disableIRQ) __enable_irq();
            return DHTLIB_ERROR_TIMEOUT_C;
        }

        // 高电平宽度决定 0/1：26~28µs = 0，70µs = 1
        // 轮询计数 ≈ µs（每轮 delay_us(1)）
        uint32_t t = 0;
        while (_pin.read() == Hig)
        {
            t++;
            if (t > TIMEOUT_BIT) break;
            delay_us(1);
        }
        if (_pin.read() == Hig)
        {
            if (_disableIRQ) __enable_irq();
            return DHTLIB_ERROR_TIMEOUT_D;
        }

        if (t > DHTLIB_BIT_THRESHOLD)
        {
            _bits[idx] |= mask;
        }

        // 推进到下一个位
        mask >>= 1;
        if (mask == 0)
        {
            mask = 0x80;
            idx++;
        }
    }
    if (_disableIRQ) __enable_irq();

    // 湿度最高位不允许为 1（DHT22 湿度最大 1000 = 0x03E8；
    // DHT11 最大 0x6400）——捕获 ESP 单位移位 bug（参考库同款检查）
    if (_bits[0] & 0x80)
    {
        return DHTLIB_ERROR_BIT_SHIFT;
    }

    return DHTLIB_OK;
}


//  返回 true 表示超时；state 被观察到 2 次即认为到达（参考库防毛刺）
bool DeviceDHTNEW::_waitFor(uint8_t state, uint32_t timeout)
{
    uint32_t count = 2;
    for (uint32_t i = 0; i < timeout; i++)
    {
        if ((_pin.read() == Hig) == (state != 0))
        {
            count--;
            if (count == 0) return false;
        }
        delay_us(1);
    }
    return true;
}


// ────────────────────────────────────────────────────────
//  低功耗
// ────────────────────────────────────────────────────────

void DeviceDHTNEW::powerUp()
{
    _pinOut();
    _pin.high();
    // 空读一次与传感器同步（参考库 powerUp()）
    read();
}


void DeviceDHTNEW::powerDown()
{
    _pinOut();
    _pin.low();
}


// ────────────────────────────────────────────────────────
//  私有：引脚模式切换（io_ctrl::reinit）
// ────────────────────────────────────────────────────────

void DeviceDHTNEW::_pinOut()
{
    if (_pin.is_initialized())
        _pin.reinit(mode_out_pp, nopull, speed_low);
    else
        _pin.init(mode_out_pp, nopull, speed_low);
}


void DeviceDHTNEW::_pinIn()
{
    if (_pin.is_initialized())
        _pin.reinit(mode_input, pullup);
    else
        _pin.init(mode_input, pullup);
}


//  -- END OF FILE --
