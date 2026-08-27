#include "device_rotarydecoder.hpp"
#include "systick.h"

// ============================================================
//  基于 PCF8574 的旋转编码器驱动
//  来源：Rob Tillaart 的 Arduino 库 "rotaryDecoder" v0.4.1
//    URL：https://github.com/RobTillaart/rotaryDecoder
//  移植说明见 device_rotarydecoder.hpp 头注释
// ============================================================

// ============================================================
//  构造 / 初始化
// ============================================================

rotaryDecoder::rotaryDecoder(PCF8574& pcf, uint8_t encoderIndex)
    : _pcf(pcf)
    , _index(encoderIndex > (MAX_COUNT - 1) ? (MAX_COUNT - 1) : encoderIndex)
    , _pinA((uint8_t)(2 * _index))
    , _pinB((uint8_t)(2 * _index + 1))
    , _lastPos(0)
    , _value(0)
    , _switchPin(SWITCH_NONE)
    , _keyPressTime(50)
    , _keyStable(1)          // 初始视为松开
    , _keyChangeTick(get_tick())
{
}

void rotaryDecoder::init()
{
    // 对应参考库 readInitialState()：只取本编码器 2 位作基准，不计数
    _lastPos = (uint8_t)((_pcf.read8() >> (2 * _index)) & 0x03);
}

// ============================================================
//  轮询（对应参考库 update() 的单编码器版本）
//  方向判定表与参考库一致：
//    +1: 0b0001 0b0111 0b1110 0b1000
//    -1: 0b0010 0b0100 0b1101 0b1011
// ============================================================

bool rotaryDecoder::poll()
{
    uint8_t value      = _pcf.read8();                                  // 一次 I2C 读
    uint8_t currentPos = (uint8_t)((value >> (2 * _index)) & 0x03);

    if (currentPos == _lastPos)
    {
        return false;
    }

    bool changed = false;
    uint8_t change = (uint8_t)((_lastPos << 2) | currentPos);
    switch (change)
    {
        case 0b0001:   // 逐位变化沿：顺时针
        case 0b0111:
        case 0b1110:
        case 0b1000:
            _value++;
            changed = true;
            break;
        case 0b0010:   // 逆时针
        case 0b0100:
        case 0b1101:
        case 0b1011:
            _value--;
            changed = true;
            break;
        default:       // 跳变（抖动/非法沿）：只更新基准不计数
            break;
    }
    _lastPos = currentPos;
    return changed;
}

// ============================================================
//  计数
// ============================================================

bool rotaryDecoder::setValue(int32_t value)
{
    _value = value;
    return true;
}

void rotaryDecoder::reset()
{
    _lastPos = 0;
    _value   = 0;
}

// ============================================================
//  按键（简化版消抖）
//  电平变化后需稳定 _keyPressTime ms 才认定为有效状态。
// ============================================================

void rotaryDecoder::setSwitchPin(uint8_t pin)
{
    _switchPin = pin;
    _keyStable = 1;                       // 重新配置后视为松开
    _keyChangeTick = get_tick();
}

bool rotaryDecoder::isKeyPressed()
{
    if (_switchPin == SWITCH_NONE)
    {
        return false;
    }

    // 按下 = 低电平（编码器按键常见接法，见头注释）
    uint8_t level = (_pcf.read(_switchPin) == SET) ? 1u : 0u;
    uint32_t now  = get_tick();

    if (level != _keyStable)
    {
        _keyStable     = level;           // 电平变化：重新计时
        _keyChangeTick = now;
        return false;
    }
    if (now - _keyChangeTick >= _keyPressTime)
    {
        return (level == 0);              // 稳定超过消抖时间才有效
    }
    return false;
}
