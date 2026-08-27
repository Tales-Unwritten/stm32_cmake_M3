#pragma once
//
//    FILE: device_hc4052.hpp
//  AUTHOR: Rob Tillaart
//    DATE: 2023-01-25
// VERSION: 0.3.2
// PURPOSE: CD74HC4052 dual 4-channel analog multiplexer（双 4 通道模拟开关）
//     URL: https://github.com/RobTillaart/HC4052
//
//  移植说明（Arduino → STM32 项目风格）：
//   - header-only，类名/API 与参考库一致
//   - 引脚参数由 uint8_t 引脚号改为 io_ctrl& 引用，可选使能脚用
//     io_ctrl* = nullptr（对应参考库 255）
//   - 参考库直接写 A/B 两引脚（无只写变化位优化），原样保留
//   - 使能脚低有效（enable() 拉低 = 通道导通）

#ifdef __cplusplus

#include "inter_io_ctrl.hpp"
#include <cstdint>

class HC4052
{
public:

    explicit HC4052(io_ctrl &A, io_ctrl &B, io_ctrl *enablePin = nullptr)
        : _pins{&A, &B}
        , _enablePin(enablePin)
        , _channel(0)
    {
        for (auto *p : _pins) _init_out(*p);
        if (_enablePin) _init_out(*_enablePin);

        for (auto *p : _pins) p->low();
        if (_enablePin) _enablePin->high();   // 默认禁能（高 = 断开）
    }

    /// 选择通道 0..3（两路开关同步切换；非法返回 false）
    bool setChannel(uint8_t channel, bool disable = true)
    {
        if (channel > 3) return false;
        if (disable) this->disable();       // 防鬼影
        _pins[0]->set((channel & 0x01) != 0);
        _pins[1]->set((channel & 0x02) != 0);
        if (disable) this->enable();
        _channel = channel;
        return true;
    }

    uint8_t getChannel() { return _channel; }

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

    io_ctrl *_pins[2];
    io_ctrl *_enablePin;
    uint8_t  _channel;
};

#endif /* __cplusplus */
