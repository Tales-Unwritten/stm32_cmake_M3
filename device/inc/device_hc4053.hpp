#pragma once
//
//    FILE: device_hc4053.hpp
//  AUTHOR: Rob Tillaart
//    DATE: 2023-01-25
// VERSION: 0.3.2
// PURPOSE: CD74HC4053 triple 2-channel analog multiplexer（三路独立 2 通道开关）
//     URL: https://github.com/RobTillaart/HC4053
//
//  移植说明（Arduino → STM32 项目风格）：
//   - header-only，类名/API 与参考库一致
//   - 引脚参数由 uint8_t 引脚号改为 io_ctrl& 引用，可选使能脚用
//     io_ctrl* = nullptr（对应参考库 255）
//   - A/B/C 三路开关可独立控制（setChannelA/B/C），也可同步设置
//   - 使能脚低有效（enable() 拉低 = 通道导通）

#ifdef __cplusplus

#include "inter_io_ctrl.hpp"
#include <cstdint>

class HC4053
{
public:

    explicit HC4053(io_ctrl &A, io_ctrl &B, io_ctrl &C, io_ctrl *enablePin = nullptr)
        : _pins{&A, &B, &C}
        , _enablePin(enablePin)
        , _channelA(0), _channelB(0), _channelC(0)
    {
        for (auto *p : _pins) _init_out(*p);
        if (_enablePin) _init_out(*_enablePin);

        for (auto *p : _pins) p->low();
        if (_enablePin) _enablePin->high();   // 默认禁能（高 = 断开）
    }

    /// 三路同步设置为同一通道（0..1），非法返回 false
    bool setChannel(uint8_t channel, bool disable = true)
    {
        if (channel > 1) return false;
        setChannelA(channel);
        setChannelB(channel);
        setChannelC(channel);
        if (disable)
        {
            this->disable();
            this->enable();
        }
        return true;
    }

    /// 仅设置 A 路（0..1）
    void setChannelA(uint8_t channel)
    {
        _channelA = channel & 0x01;
        _pins[0]->set(_channelA != 0);
    }

    /// 仅设置 B 路（0..1）
    void setChannelB(uint8_t channel)
    {
        _channelB = channel & 0x01;
        _pins[1]->set(_channelB != 0);
    }

    /// 仅设置 C 路（0..1）
    void setChannelC(uint8_t channel)
    {
        _channelC = channel & 0x01;
        _pins[2]->set(_channelC != 0);
    }

    uint8_t getChannelA() { return _channelA; }
    uint8_t getChannelB() { return _channelB; }
    uint8_t getChannelC() { return _channelC; }

    /// 使能（拉低 = 通道导通）
    void enable()
    {
        if (_enablePin) _enablePin->low();
    }

    /// 禁能（拉高 = 全部断开）
    void disable()
    {
        if (_enablePin) _enablePin->high();
    }

    bool isEnabled()
    {
        if (!_enablePin) return true;
        return _enablePin->read() == Low;
    }

private:

    static void _init_out(io_ctrl &p)
    {
        if (p.is_initialized())
            p.reinit(mode_out_pp, nopull, speed_low);
        else
            p.init(mode_out_pp, nopull, speed_low);
    }

    io_ctrl *_pins[3];
    io_ctrl *_enablePin;
    uint8_t  _channelA;
    uint8_t  _channelB;
    uint8_t  _channelC;
};

#endif /* __cplusplus */
