#pragma once
//
//    FILE: device_shiftout.hpp
//  AUTHOR: Rob Tillaart
// VERSION: 0.4.3（参考库 FastShiftOut）
//    DATE: 2013-08-22
// PURPOSE: 软件移位输出（74HC595 等串行扩展输出）
//     URL: https://github.com/RobTillaart/FastShiftOut
//
//  移植说明（Arduino → STM32 项目风格）：
//   - header-only，类名 DeviceShiftOut；参考库实现 Print 接口
//     （write(uint8_t)），本移植改为 write(data, bits) 直通位宽
//   - 参考库只有 dataOut + clockPin 两根线；本移植增加可选
//     latchPin（74HC595 RCLK，写完后自动输出上升沿锁存），
//     不需要锁存脚时传 nullptr（对应 74HC595 直连输出）
//   - 引脚由 uint8_t 改为 io_ctrl& 引用（可选锁存脚 io_ctrl*）
//   - 软件移位不延时（Arduino shiftOut 同款；74HC595 SRCLK 最高
//     20MHz+，speed_low 的边沿速度远在其范围内），
//     若需降低时钟速率可在 _pulseClock() 中自行加 delay_us(1)
//   - 位序：LSBFIRST = 先发最低位，MSBFIRST = 先发最高位
//     （参考库默认 LSBFIRST）

#ifdef __cplusplus

#include "inter_io_ctrl.hpp"
#include "systick.h"   // delay_us()
#include <cstdint>

class DeviceShiftOut
{
public:
    // ── 位序（对应 Arduino LSBFIRST / MSBFIRST）─────────

    static constexpr uint8_t LSBFIRST = 0;
    static constexpr uint8_t MSBFIRST = 1;

    /**
     * @brief 构造并初始化引脚（推挽输出；时钟空闲低，上升沿移入）
     * @param dataPin   串行数据脚（74HC595 DS/SER）
     * @param clockPin  移位时钟脚（74HC595 SHCP/SRCLK）
     * @param latchPin  锁存脚（74HC595 STCP/RCLK），nullptr = 无锁存
     * @param bitOrder  LSBFIRST / MSBFIRST
     */
    explicit DeviceShiftOut(io_ctrl &dataPin, io_ctrl &clockPin,
                            io_ctrl *latchPin = nullptr,
                            uint8_t bitOrder = LSBFIRST)
        : _data(dataPin)
        , _clock(clockPin)
        , _latch(latchPin)
        , _bitOrder(bitOrder)
        , _lastValue(0)
    {
        _init_out(_data);
        _init_out(_clock);
        _clock.low();   // 空闲低，上升沿把数据移入移位寄存器
        if (_latch)
        {
            _init_out(*_latch);
            _latch->low();   // RCLK 空闲低，写完后上升沿锁存到输出
        }
    }

    // ── 输出 ────────────────────────────────────────────

    /**
     * @brief 软件移位输出 data 的低 bits 位（1..32）
     * @return false = bits 非法（0 或 > 32）
     * @note  配置了锁存脚时，写完后自动输出一个锁存脉冲
     */
    bool write(uint32_t data, uint8_t bits = 8)
    {
        if (_bitOrder == LSBFIRST)
        {
            if (!_writeLSB(data, bits)) return false;
        }
        else
        {
            if (!_writeMSB(data, bits)) return false;
        }
        _latchPulse();
        return true;
    }

    // ── 便捷封装（参考库 write16/24/32）────────────────

    bool write16(uint16_t data) { return write(data, 16); }
    bool write24(uint32_t data) { return write(data, 24); }
    bool write32(uint32_t data) { return write(data, 32); }

    /** @brief 无视 bitOrder 强制最低位先出 */
    bool writeLSB(uint32_t data, uint8_t bits = 8)
    {
        if (!_writeLSB(data, bits)) return false;
        _latchPulse();
        return true;
    }

    /** @brief 无视 bitOrder 强制最高位先出 */
    bool writeMSB(uint32_t data, uint8_t bits = 8)
    {
        if (!_writeMSB(data, bits)) return false;
        _latchPulse();
        return true;
    }

    // ── 元信息 ──────────────────────────────────────────

    /** @brief 最近一次写入的字节（低 8 位，参考库 lastWritten） */
    uint8_t lastWritten() { return _lastValue; }

    /** @brief 设置位序，非法值返回 false */
    bool setBitOrder(uint8_t bitOrder)
    {
        if ((bitOrder == LSBFIRST) || (bitOrder == MSBFIRST))
        {
            _bitOrder = bitOrder;
            return true;
        }
        return false;
    }

    uint8_t getBitOrder() { return _bitOrder; }

private:
    bool _writeLSB(uint32_t data, uint8_t bits)
    {
        if ((bits == 0) || (bits > 32)) return false;
        _lastValue = (uint8_t)data;
        for (uint8_t i = 0; i < bits; i++)
        {
            _data.set((data & 1u) != 0);
            _pulseClock();
            data >>= 1;
        }
        return true;
    }

    bool _writeMSB(uint32_t data, uint8_t bits)
    {
        if ((bits == 0) || (bits > 32)) return false;
        _lastValue = (uint8_t)data;
        for (uint8_t i = 0; i < bits; i++)
        {
            _data.set((data & (1u << (bits - 1 - i))) != 0);
            _pulseClock();
        }
        return true;
    }

    void _pulseClock()
    {
        // 上升沿移入 74HC595 移位寄存器；无延时（Arduino shiftOut 同款）
        _clock.high();
        _clock.low();
    }

    void _latchPulse()
    {
        if (_latch)
        {
            _latch->high();   // 上升沿把移位寄存器锁存到输出
            delay_us(1);
            _latch->low();
        }
    }

    static void _init_out(io_ctrl &p)
    {
        if (p.is_initialized())
            p.reinit(mode_out_pp, nopull, speed_low);
        else
            p.init(mode_out_pp, nopull, speed_low);
    }

    io_ctrl &_data;
    io_ctrl &_clock;
    io_ctrl *_latch;
    uint8_t  _bitOrder;
    uint8_t  _lastValue;
};

#endif /* __cplusplus */
