#pragma once
//
//    FILE: device_74hc154.hpp
//  AUTHOR: Rob Tillaart
//    DATE: 2024-09-08
// VERSION: 0.2.1
// PURPOSE: 74HC154 4-to-16 line decoder（4 位地址线 → 16 路选通输出）
//     URL: https://github.com/RobTillaart/74HC154
//
//  移植说明（Arduino → STM32 项目风格）：
//   - header-only，类名/API 与参考库一致
//   - 引脚参数由 uint8_t 引脚号改为 io_ctrl& 引用，可选使能脚用
//     io_ctrl* = nullptr（对应参考库 255）
//   - 构造时自动把引脚初始化为推挽输出并置默认电平
//   - 使能脚低有效（enable() 拉低），与参考库一致

#ifdef __cplusplus

#include "inter_io_ctrl.hpp"
#include <cstdint>

class DEV_74HC154
{
public:

    explicit DEV_74HC154(io_ctrl &pin0, io_ctrl &pin1, io_ctrl &pin2, io_ctrl &pin3,
                         io_ctrl *pinEnable = nullptr)
        : _pin{&pin0, &pin1, &pin2, &pin3}
        , _enable(pinEnable)
        , _line(0)
    {
        for (auto *p : _pin) _init_out(*p);
        if (_enable) _init_out(*_enable);

        for (auto *p : _pin) p->low();
        if (_enable) _enable->high();   // 默认禁能（高）
    }

    // 四引脚数组版（对应参考库 pins[] 构造；传入 io_ctrl* 指针数组）
    explicit DEV_74HC154(io_ctrl **pins, io_ctrl *pinEnable = nullptr)
        : _pin{pins[0], pins[1], pins[2], pins[3]}
        , _enable(pinEnable)
        , _line(0)
    {
        for (auto *p : _pin) _init_out(*p);
        if (_enable) _init_out(*_enable);

        for (auto *p : _pin) p->low();
        if (_enable) _enable->high();
    }

    // ── 选通输出 ──────────────────────────────────────────

    /// 选择第 line 路输出（0..15），非法返回 false
    bool setLine(uint8_t line)
    {
        if (line > 15) return false;
        _line = line;
        _pin[0]->set((line & 0x01) != 0);
        _pin[1]->set((line & 0x02) != 0);
        _pin[2]->set((line & 0x04) != 0);
        _pin[3]->set((line & 0x08) != 0);
        return true;
    }

    uint8_t getLine() { return _line; }

    /// 下一路（0..15 循环）
    void nextLine()
    {
        if (_line >= 15) _line = 0;
        else _line++;
        _setLine();
    }

    /// 上一路（0→15 回绕）
    void prevLine()
    {
        if (_line == 0) _line = 15;
        else _line--;
        _setLine();
    }

    // ── 使能（E1/E2 低有效，拉低 = 使能） ──────────────────

    void enable()
    {
        if (_enable) _enable->low();
    }

    void disable()
    {
        if (_enable) _enable->high();
    }

    bool isEnabled()
    {
        if (!_enable) return true;   // 未接使能脚视为常使能
        return _enable->read() == Low;
    }

private:

    static void _init_out(io_ctrl &p)
    {
        if (p.is_initialized())
            p.reinit(mode_out_pp, nopull, speed_low);
        else
            p.init(mode_out_pp, nopull, speed_low);
    }

    void _setLine()
    {
        _pin[0]->set((_line & 0x01) != 0);
        _pin[1]->set((_line & 0x02) != 0);
        _pin[2]->set((_line & 0x04) != 0);
        _pin[3]->set((_line & 0x08) != 0);
    }

    io_ctrl *_pin[4];
    io_ctrl *_enable;
    uint8_t  _line;
};

#endif /* __cplusplus */
