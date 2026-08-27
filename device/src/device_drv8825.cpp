#include "device_drv8825.hpp"
#include "systick.h"

// ============================================================
//  DRV8825 步进电机驱动
//  来源：Rob Tillaart 的 Arduino 库 "DRV8825" v0.2.2
//    URL：https://github.com/RobTillaart/DRV8825
//  移植说明见 device_drv8825.hpp 头注释
// ============================================================

// ============================================================
//  构造 / 初始化
// ============================================================

DRV8825::DRV8825(io_ctrl& step, io_ctrl& dir, io_ctrl& en,
                 io_ctrl* rst, io_ctrl* slp,
                 io_ctrl* m0, io_ctrl* m1, io_ctrl* m2)
    : _step(step)
    , _dir(dir)
    , _en(en)
    , _rst(rst)
    , _slp(slp)
    , _m0(m0)
    , _m1(m1)
    , _m2(m2)
    , _direction(CLOCK_WISE)
    , _stepsPerRevolution(0)
    , _steps(0)
    , _position(0)
    , _stepPulseLength(2)
{
}

void DRV8825::init()
{
    _step.low();
    _dir.low();
    _en.low();                       // EN 低有效：默认使能

    if (_rst) _rst->high();          // RST 低有效：默认非复位
    if (_slp) _slp->high();          // SLP 低有效：默认唤醒

    // 微步引脚默认全步进（M2 M1 M0 = 0 0 0）
    if (_m0) _m0->low();
    if (_m1) _m1->low();
    if (_m2) _m2->low();
}

// ============================================================
//  方向
// ============================================================

bool DRV8825::setDirection(uint8_t direction)
{
    if (direction > 1)
    {
        return false;
    }
    _direction = direction;
    // 数据手册：DIR 建立/保持时间 ≥ 650ns
    delay_us(1);
    _dir.set(_direction != 0);
    delay_us(1);
    return true;
}

uint8_t DRV8825::getDirection()
{
    return (_dir.read() == Hig) ? COUNTERCLOCK_WISE : CLOCK_WISE;
}

// ============================================================
//  步进
// ============================================================

void DRV8825::setStepsPerRevolution(uint16_t stepsPerRevolution)
{
    _stepsPerRevolution = stepsPerRevolution;
}

void DRV8825::step()
{
    _step.high();
    if (_stepPulseLength > 0)
    {
        delay_us(_stepPulseLength);
    }
    _step.low();
    if (_stepPulseLength > 0)
    {
        delay_us(_stepPulseLength);
    }

    _steps++;
    if (_stepsPerRevolution > 0)
    {
        // 圈内位置跟踪（与参考库一致：顺时针递增，逆时针递减）
        if (_direction == CLOCK_WISE)
        {
            _position++;
            if (_position >= _stepsPerRevolution)
            {
                _position = 0;
            }
        }
        else
        {
            if (_position == 0)
            {
                _position = _stepsPerRevolution;
            }
            _position--;
        }
    }
}

void DRV8825::stepMotor(uint32_t steps, uint8_t direction)
{
    if (!setDirection(direction))
    {
        return;                      // 方向参数非法：不动作
    }
    while (steps--)
    {
        step();
    }
}

uint32_t DRV8825::resetSteps(uint32_t s)
{
    uint32_t t = _steps;
    _steps = s;
    return t;
}

// ============================================================
//  位置 / 脉冲宽度
// ============================================================

bool DRV8825::setPosition(uint16_t position)
{
    if (position >= _stepsPerRevolution)
    {
        return false;                // 参考库语义：0 圈时任何值都失败
    }
    _position = position;
    return true;
}

void DRV8825::setStepPulseLength(uint16_t stepPulseLength)
{
    _stepPulseLength = stepPulseLength;
}

// ============================================================
//  使能 / 复位 / 休眠（均低有效）
// ============================================================

bool DRV8825::enable()
{
    _en.low();
    return true;
}

bool DRV8825::disable()
{
    _en.high();
    return true;
}

bool DRV8825::isEnabled()
{
    return (_en.read() == Low);    // 低有效
}

bool DRV8825::reset()
{
    if (!_rst)
    {
        return false;
    }
    _rst->low();
    delay_ms(1);
    _rst->high();
    return true;
}

bool DRV8825::sleep()
{
    if (!_slp)
    {
        return false;
    }
    _slp->low();
    return true;
}

bool DRV8825::wakeup()
{
    if (!_slp)
    {
        return false;
    }
    _slp->high();
    return true;
}

bool DRV8825::isSleeping()
{
    return (_slp != nullptr) && (_slp->read() == Low);
}

// ============================================================
//  微步（M2 M1 M0 真值表）
// ============================================================

bool DRV8825::setMicrostep(Microstep ms)
{
    if (!_m0 || !_m1 || !_m2)
    {
        return false;
    }
    uint8_t bits = (uint8_t)ms;      // bit0=M0 bit1=M1 bit2=M2
    _m0->set((bits & 0x01) != 0);
    _m1->set((bits & 0x02) != 0);
    _m2->set((bits & 0x04) != 0);
    return true;
}
