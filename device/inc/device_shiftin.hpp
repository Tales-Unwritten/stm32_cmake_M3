#pragma once
//
//    FILE: device_shiftin.hpp
//  AUTHOR: Rob Tillaart
// VERSION: 0.4.2（参考库 FastShiftIn）
//    DATE: 2013-09-29
// PURPOSE: 软件移位输入（74HC165 等串行扩展输入）
//     URL: https://github.com/RobTillaart/FastShiftIn
//
//  移植说明（Arduino → STM32 项目风格）：
//   - header-only，类名 DeviceShiftIn；参考库 read() 固定 8 位，
//     本移植改为 read(bits) 直通位宽（1..32）
//   - 参考库只有 dataIn + clockPin 两根线；本移植增加可选
//     latchPin（74HC165 SH/LD，读前自动拉低装载并行数据），
//     不需要装载脚时传 nullptr（如数据已由外部装载）
//   - 引脚由 uint8_t 改为 io_ctrl& 引用（可选装载脚 io_ctrl*）
//   - 时钟带 1µs 延时（Arduino shiftIn 同款）：74HC165 在时钟
//     上升沿把数据移出，读取需等待输出稳定（t_PD ≈ 12~40ns）
//   - 数据脚配置为输入（74HC165 输出为推挽，无需上拉）
//   - 位序：LSBFIRST = 先收最低位，MSBFIRST = 先收最高位
//     （参考库默认 LSBFIRST）

#ifdef __cplusplus

#include "inter_io_ctrl.hpp"
#include "systick.h"   // delay_us()
#include <cstdint>

class DeviceShiftIn
{
public:
    // ── 位序（对应 Arduino LSBFIRST / MSBFIRST）─────────

    static constexpr uint8_t LSBFIRST = 0;
    static constexpr uint8_t MSBFIRST = 1;

    /**
     * @brief 构造并初始化引脚
     * @param dataPin   串行数据脚（74HC165 Q7/SO，输入）
     * @param clockPin  移位时钟脚（74HC165 CP/CLK，推挽输出）
     * @param latchPin  并行装载脚（74HC165 SH/LD，低有效），nullptr = 无
     * @param bitOrder  LSBFIRST / MSBFIRST
     */
    explicit DeviceShiftIn(io_ctrl &dataPin, io_ctrl &clockPin,
                           io_ctrl *latchPin = nullptr,
                           uint8_t bitOrder = LSBFIRST)
        : _data(dataPin)
        , _clock(clockPin)
        , _latch(latchPin)
        , _bitOrder(bitOrder)
        , _lastValue(0)
    {
        if (_data.is_initialized())
            _data.reinit(mode_input, nopull);
        else
            _data.init(mode_input, nopull);

        _init_out(_clock);
        _clock.low();   // 空闲低，上升沿移出数据
        if (_latch)
        {
            _init_out(*_latch);
            _latch->high();   // SH/LD 空闲高；拉低 = 装载并行输入
        }
    }

    // ── 读取 ────────────────────────────────────────────

    /**
     * @brief 读入 bits 位（1..32），非法位宽返回 0
     * @note  配置了装载脚时，读前自动执行 SH/LD 拉低装载并行数据
     */
    uint32_t read(uint8_t bits = 8)
    {
        if ((bits == 0) || (bits > 32)) return 0;

        _latchLoad();

        uint32_t value = 0;
        if (_bitOrder == LSBFIRST)
        {
            for (uint8_t i = 0; i < bits; i++)
            {
                if (_readBit()) value |= (1u << i);
            }
        }
        else
        {
            for (uint8_t i = 0; i < bits; i++)
            {
                if (_readBit()) value |= (1u << (bits - 1 - i));
            }
        }

        _lastValue = value;
        return value;
    }

    // ── 便捷封装（参考库 read16/24/32）────────────────

    uint32_t read16() { return read(16); }
    uint32_t read24() { return read(24); }
    uint32_t read32() { return read(32); }

    /** @brief 无视 bitOrder 强制最低位先收 */
    uint32_t readLSB(uint8_t bits = 8)
    {
        if ((bits == 0) || (bits > 32)) return 0;
        _latchLoad();
        uint32_t value = 0;
        for (uint8_t i = 0; i < bits; i++)
        {
            if (_readBit()) value |= (1u << i);
        }
        _lastValue = value;
        return value;
    }

    /** @brief 无视 bitOrder 强制最高位先收 */
    uint32_t readMSB(uint8_t bits = 8)
    {
        if ((bits == 0) || (bits > 32)) return 0;
        _latchLoad();
        uint32_t value = 0;
        for (uint8_t i = 0; i < bits; i++)
        {
            if (_readBit()) value |= (1u << (bits - 1 - i));
        }
        _lastValue = value;
        return value;
    }

    // ── 元信息 ──────────────────────────────────────────

    /** @brief 最近一次读取的值（参考库 lastRead） */
    uint32_t lastRead() { return _lastValue; }

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
    // 装载并行输入（74HC165 SH/LD 低有效），无装载脚时为空操作
    void _latchLoad()
    {
        if (_latch)
        {
            _latch->low();
            delay_us(1);
            _latch->high();
            delay_us(1);
        }
    }

    //  时钟上升沿移出数据，延时 1µs 后采样（Arduino shiftIn 同款）
    bool _readBit()
    {
        _clock.high();
        delay_us(1);
        bool bit = (_data.read() == Hig);
        _clock.low();
        delay_us(1);
        return bit;
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
    uint32_t _lastValue;
};

#endif /* __cplusplus */
