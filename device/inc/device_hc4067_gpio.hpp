#pragma once
//
//    FILE: device_hc4067_gpio.hpp
//  AUTHOR: Rob Tillaart
//    DATE: 2023-01-25
// VERSION: 0.3.2
// PURPOSE: CD74HC4067 1 x 16 channel multiplexer（GPIO 直连版）
//     URL: https://github.com/RobTillaart/HC4067
//
//  移植说明（Arduino → STM32 项目风格）：
//   - header-only，类名/API 与参考库一致
//   - 引脚参数由 uint8_t 引脚号改为 io_ctrl& 引用，可选使能脚用
//     io_ctrl* = nullptr（对应参考库 255）
//   - 保留参考库"只写变化位 + 切换前禁能防鬼影"逻辑
//   - 使能脚低有效（enable() 拉低 = 通道导通）
//
//  ⚠️ 与 device_hc4067.hpp（I2C 版，PCF8574 控制）的区别：
//     本文件为 MCU GPIO 直连 S0..S3 的版本，类名 HC4067_GPIO；
//     I2C 版通过 I2C 总线写 IO 扩展器控制，类名 HC4067。按需选用。

#ifdef __cplusplus

#include "inter_io_ctrl.hpp"
#include <cstdint>

class HC4067_GPIO
{
public:

    explicit HC4067_GPIO(io_ctrl &s0, io_ctrl &s1, io_ctrl &s2, io_ctrl &s3,
                         io_ctrl *enablePin = nullptr)
        : _pins{&s0, &s1, &s2, &s3}
        , _enablePin(enablePin)
        , _channel(0)
    {
        for (auto *p : _pins) _init_out(*p);
        if (_enablePin) _init_out(*_enablePin);

        for (auto *p : _pins) p->low();
        if (_enablePin) _enablePin->high();   // 默认禁能（高 = 断开）
    }

    /// 选择通道 0..15（非法返回 false）
    /// disable=true 时切换前先禁能，防止鬼影通道
    bool setChannel(uint8_t channel, bool disable = true)
    {
        if (channel > 15) return false;
        if (channel != _channel)
        {
            uint8_t changed = channel ^ _channel;
            uint8_t mask = 0x08;
            int8_t  i = 3;
            if (disable) this->disable();     // 防鬼影
            while (mask)
            {
                // 只写变化的位
                if (mask & changed)
                {
                    _pins[i]->set((mask & channel) != 0);
                }
                i--;
                mask >>= 1;
            }
            this->enable();
            _channel = channel;
        }
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

    io_ctrl *_pins[4];
    io_ctrl *_enablePin;
    uint8_t  _channel;
};

#endif /* __cplusplus */
