#pragma once
//
//    FILE: device_74hc590.hpp
//  AUTHOR: Rob Tillaart
//    DATE: 2025-04-30
// VERSION: 0.1.1
// PURPOSE: 54HC590 / 74HC590 8-bit binary counter（二进制计数器）
//     URL: https://github.com/RobTillaart/74HC590
//
//  移植说明（Arduino → STM32 项目风格）：
//   - header-only，类名/API 与参考库一致
//   - 引脚参数由 uint8_t 引脚号改为 io_ctrl& 引用
//   - RCLK（寄存器锁存时钟）与 RCO（进位输出，输入）可选，
//     不使用时传 nullptr（对应参考库 255）
//   - RCLK 不接时寄存器随 CCLK 直通（参考库语义：RCLK == CCLK）

#ifdef __cplusplus

#include "inter_io_ctrl.hpp"
#include <cstdint>

class DEV_74HC590
{
public:

    //  OE      output enable（低有效）
    //  CCLR    counter clear（上升沿清零）
    //  CCKEN   counter clock enable（低有效）
    //  CCLK    counter clock
    //  RCLK    register clock，nullptr 表示 RCLK 与 CCLK 共用
    //  RCO     ripple carry out（输入），nullptr 表示不读
    explicit DEV_74HC590(io_ctrl &OE, io_ctrl &CCLR, io_ctrl &CCKEN, io_ctrl &CCLK,
                         io_ctrl *RCLK = nullptr, io_ctrl *RCO = nullptr)
        : _OE(OE), _CCLR(CCLR), _CCKEN(CCKEN), _CCLK(CCLK)
        , _RCLK(RCLK), _RCO(RCO)
    {
        _init_out(_OE);
        _init_out(_CCLR);
        _init_out(_CCKEN);
        _init_out(_CCLK);
        if (_RCLK) _init_out(*_RCLK);
        if (_RCO)
        {
            if (_RCO->is_initialized())
                _RCO->reinit(mode_input, nopull);
            else
                _RCO->init(mode_input, nopull);
        }

        _OE.low();      // 默认使能输出
        _CCLR.low();
        _CCKEN.low();   // 默认使能计数
        _CCLK.low();
        if (_RCLK) _RCLK->low();
    }

    // ── 输出使能（OE 低有效） ──────────────────────────────

    void enableOutput()  { _OE.low(); }
    void disableOutput() { _OE.high(); }

    // ── 计数控制 ──────────────────────────────────────────

    /// 清零计数器（CCLR 上升沿触发）
    void clearCounter()
    {
        _CCLR.low();
        _CCLR.high();
    }

    /// 使能计数（CCKEN 低有效）
    void enableCounter()  { _CCKEN.low(); }
    void disableCounter() { _CCKEN.high(); }

    /// 计数器时钟脉冲（CCLK 高→低）
    void pulseCounter()
    {
        _CCLK.high();
        _CCLK.low();
    }

    /// 寄存器锁存脉冲（RCLK 高→低；未接 RCLK 时无操作）
    void pulseRegister()
    {
        if (_RCLK == nullptr) return;
        _RCLK->high();
        _RCLK->low();
    }

    /// 读进位输出 RCO（1 = 有进位/溢出；未接时返回 0）
    uint8_t readRCO()
    {
        if (_RCO == nullptr) return 0;
        return (_RCO->read() == Hig) ? 1 : 0;
    }

protected:

    static void _init_out(io_ctrl &p)
    {
        if (p.is_initialized())
            p.reinit(mode_out_pp, nopull, speed_low);
        else
            p.init(mode_out_pp, nopull, speed_low);
    }

    io_ctrl &_OE;
    io_ctrl &_CCLR;
    io_ctrl &_CCKEN;
    io_ctrl &_CCLK;
    io_ctrl *_RCLK;
    io_ctrl *_RCO;
};

// ============================================================
//  派生类：54HC590（与 74HC590 完全兼容，仅型号命名区分）
// ============================================================

class DEV_54HC590 : public DEV_74HC590
{
public:
    explicit DEV_54HC590(io_ctrl &OE, io_ctrl &CCLR, io_ctrl &CCKEN, io_ctrl &CCLK,
                         io_ctrl *RCLK = nullptr, io_ctrl *RCO = nullptr)
        : DEV_74HC590(OE, CCLR, CCKEN, CCLK, RCLK, RCO)
    {
    }
};

#endif /* __cplusplus */
