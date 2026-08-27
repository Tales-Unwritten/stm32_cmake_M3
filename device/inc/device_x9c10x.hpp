#pragma once
//
//    FILE: device_x9c10x.hpp
//  AUTHOR: Rob Tillaart
// VERSION: 0.2.5（参考库 X9C10X）
//    DATE: 2022-01-26
// PURPOSE: X9C10X 系列数字电位器（INC / UD / CS 三线，0~99 档）
//     URL: https://github.com/RobTillaart/X9C10X
//
//  移植说明（Arduino → STM32 项目风格）：
//   - header-only，类名 DeviceX9C10X；参考库的 X9C 基类与
//     X9C102/103/104/503 派生类未移植（派生类仅预设默认阻值，
//     用构造参数 maxOhm 代替）
//   - 引脚由 uint8_t 改为 io_ctrl& 引用（INC / UD / CS 三线）
//   - 无浮点换算：getOhm() / ohm2Position() 为纯整数舍入
//   - 参考库在 AVR 上时钟延时为 0（digitalWrite 本身够慢），
//     非 AVR 平台取 1µs；本移植固定 1µs
//     ★ 真机需按实测调参 ★（INC 脉宽 / Tdi / Tcph）
//   - 初始化顺序参考参考库 issue #7：避免引入意外的 STORE 脉冲
//     （先把三根线都置为输出低，再依次拉高 CS → INC → UD，
//      保证 CS 上升沿发生时 INC 为低，不会触发 EEPROM 存储）
//   - 注意：X9C10X 无法回读滑臂位置，getPosition() 返回的是
//     本机缓存；若上电后器件内部位置未知，用
//     setPosition(pos, forced=true) 先归位到端点再精确到位

#ifdef __cplusplus

#include "inter_io_ctrl.hpp"
#include "systick.h"   // delay_us() / delay_ms()
#include <cstdint>

class DeviceX9C10X
{
public:
    // ── 方向 ────────────────────────────────────────────

    static constexpr uint8_t UP   = 1;   // UD 高 = 阻值增大
    static constexpr uint8_t DOWN = 0;   // UD 低 = 阻值减小

    // ── 档位范围 ────────────────────────────────────────

    static constexpr uint8_t MIN_POSITION = 0;
    static constexpr uint8_t MAX_POSITION = 99;

    /**
     * @brief 构造并初始化三线引脚（推挽输出，空闲 CS/INC/UD 全高）
     * @param pulsePin     INC 脉冲脚
     * @param directionPin UD 方向脚
     * @param selectPin    CS 片选脚
     * @param maxOhm       最大阻值（X9C103 = 10000Ω 等，可传实测值校准）
     */
    explicit DeviceX9C10X(io_ctrl &pulsePin, io_ctrl &directionPin,
                          io_ctrl &selectPin, uint32_t maxOhm = 10000)
        : _pulse(pulsePin)
        , _direction(directionPin)
        , _select(selectPin)
        , _maxOhm(maxOhm)
        , _position(0)
        , _type(0)
    {
        // 先全部初始化为输出低电平，再按 CS → INC → UD 顺序拉高，
        // 避免 CS 上升沿时 INC 为高（= STORE 条件），见头注释
        _init_out(_pulse);
        _init_out(_direction);
        _init_out(_select);
        _select.high();
        _pulse.high();
        _direction.high();
        // 滑臂上电稳定时间（数据手册 Page 5）
        delay_us(500);
    }

    // ── 步进 ────────────────────────────────────────────

    /** @brief 增一档（到 99 档后返回 false） */
    bool incr()
    {
        if (_position >= MAX_POSITION) return false;
        _position++;
        _move(UP);
        return true;
    }

    /** @brief 减一档（到 0 档后返回 false） */
    bool decr()
    {
        if (_position == MIN_POSITION) return false;
        _position--;
        _move(DOWN);
        return true;
    }

    // ── 定位 ────────────────────────────────────────────

    /**
     * @brief 设置档位 0..99（>99 截断）
     * @param forced true 时先快速移到最近端点再精确到位（最多 99+99 步），
     *               用于器件内部位置未知时重新校准缓存；
     *               默认 false 按缓存差值移动（更快但依赖缓存正确）
     * @return 新档位 0..99
     */
    uint8_t setPosition(uint8_t position, bool forced = false)
    {
        if (position > MAX_POSITION)
        {
            position = MAX_POSITION;
        }

        // 先强制归位到最近的端点，减少步数（参考库逻辑）
        if (forced)
        {
            if (position < 50)
            {
                _move(DOWN, MAX_POSITION);
                _position = MIN_POSITION;
            }
            else
            {
                _move(UP, MAX_POSITION);
                _position = MAX_POSITION;
            }
        }

        if (position > _position)
        {
            _move(UP, position - _position);
        }
        if (position < _position)
        {
            _move(DOWN, _position - position);
        }

        _position = position;
        return _position;
    }

    /** @brief 返回缓存的档位（器件不可回读，见头注释） */
    uint8_t getPosition() { return _position; }

    // ── 存储 / 恢复 ─────────────────────────────────────

    /**
     * @brief 把当前档位写入器件内部 EEPROM（掉电保持）
     * @note  谨慎使用：EEPROM 写寿命有限（约 10 万次）
     */
    uint8_t store()
    {
        // CS 拉低再拉高，INC 为高 → 触发 STORE（数据手册）
        _select.low();
        delay_us(1);
        _select.high();
        delay_ms(20);   // Tcph ≥ 20ms（数据手册 Page 5）
        return _position;
    }

    /**
     * @brief 仅更新本机档位缓存（不移动滑臂）
     * @note  用于"上电恢复内部位置"后告知驱动当前位置（参考库同款）
     */
    uint8_t restoreInternalPosition(uint8_t position)
    {
        if (position > MAX_POSITION)
        {
            position = MAX_POSITION;
        }
        _position = position;
        return _position;
    }

    // ── 阻值换算（整数，四舍五入）──────────────────────

    /** @brief 当前档位对应阻值（Ω，线性近似） */
    uint32_t getOhm()
    {
        return (_maxOhm * _position + 49) / 99;
    }

    uint32_t getMaxOhm() { return _maxOhm; }

    /**
     * @brief 阻值 → 档位
     * @param value  目标阻值（Ω），> 最大阻值返回 99
     * @param invert true 时按反向电位器（阻值=最大-目标）换算
     */
    uint8_t ohm2Position(uint32_t value, bool invert = false)
    {
        if (value > _maxOhm) return MAX_POSITION;
        uint8_t val = (99 * value + _maxOhm / 2) / _maxOhm;
        if (invert) return MAX_POSITION - val;
        return val;
    }

    /** @brief 器件类型：X9C10X 返回 0（参考库派生类才有类型号） */
    uint16_t getType() { return _type; }

private:
    void _move(uint8_t direction, uint8_t steps = 1)
    {
        // 方向建立时间 Tdi（数据手册 Page 5）
        _direction.set(direction == UP);
        delay_us(3);

        _select.low();
        while (steps--)
        {
            _pulse.high();
            delay_us(1);   // INC 高电平脉宽（非 AVR 参考库取 1µs）
            _pulse.low();
            delay_us(1);   // INC 低电平脉宽
        }
        // 结束时 INC 为低，CS 拉高不会触发 STORE（数据手册 No Store, Page 7）
        _select.high();
        // 恢复 INC 默认高电平
        _pulse.high();
    }

    static void _init_out(io_ctrl &p)
    {
        if (p.is_initialized())
            p.reinit(mode_out_pp, nopull, speed_low);
        else
            p.init(mode_out_pp, nopull, speed_low);
    }

    io_ctrl &_pulse;
    io_ctrl &_direction;
    io_ctrl &_select;
    uint32_t _maxOhm;
    uint8_t  _position;
    uint16_t _type;
};

#endif /* __cplusplus */
