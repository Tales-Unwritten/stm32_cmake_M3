#pragma once

#ifdef __cplusplus

#include "device_pcf8574.hpp"
#include <cstdint>

// ============================================================
//  基于 PCF8574 的旋转编码器驱动
// ============================================================
//  来源：Rob Tillaart 的 Arduino 库 "rotaryDecoder"
//  版本：0.4.1（2026-01-08）
//    URL：https://github.com/RobTillaart/rotaryDecoder
//
//  接线（参考库 demo）：1 片 PCF8574 可接 4 个编码器，
//  编码器 i（0..3）的 A/B 相分别接 PCF8574 的 pin 2i / 2i+1。
//
//  移植说明（对照参考库 0.4.1）：
//    - 参考库是"一个对象管 4 个编码器"（begin/update/getValue(re)）；
//      本驱动按任务要求改为"每编码器一对象"：
//      构造传 PCF8574& + 编码器编号 0..3，poll() 由用户周期调用
//      （对应参考库 update() 的轮询模型，内部状态机一致）
//    - 方向判定表（上次状态<<2 | 本次状态）与参考库完全相同
//    - 参考库本身无按键功能（按键在 rotaryDecoderSwitch 库）；
//      本驱动按任务要求保留"简化版"按键读取：可选按键引脚 +
//      keyPressTime 消抖（HAL_GetTick 计时），按下 = 低电平
//    - 使用前提：PCF8574 需先 init()（默认 initialValue=0xFF，
//      编码器引脚呈高阻输入态）；poll() 每次做一次 I2C 读
//    - 无浮点、无堆分配、无异常
// ============================================================

class rotaryDecoder
{
public:

    static constexpr uint8_t MAX_COUNT   = 4;        // 每片 PCF8574 最多编码器数
    static constexpr uint8_t SWITCH_NONE = 0xFF;     // 按键引脚：禁用

    /**
     * @brief 构造：绑定 PCF8574 与编码器编号
     * @param pcf  已构造的 PCF8574（使用前需外部 pcf.init()）
     * @param encoderIndex  0..3（越界按 3 处理）
     */
    explicit rotaryDecoder(PCF8574& pcf, uint8_t encoderIndex);

    /** @brief 读取当前 2 位状态作为计数基准（对应参考库 readInitialState()） */
    void init();

    /**
     * @brief 周期轮询（对应参考库 update()）
     * @return true = 本编码器计数有变化
     * @note  每次调用做一次 I2C 读（read8()）
     */
    bool poll();

    // ── 计数 ──────────────────────────────────────────────

    int32_t getValue() const { return _value; }
    bool    setValue(int32_t value);          // 覆盖计数（对应参考库 setValue(re, v)）
    void    reset();                          // 清零计数与基准状态
    uint8_t getLastPosition() const { return _lastPos; }   // 最近 2 位状态（调试用）

    // ── 按键（简化版，可选；按下 = 低电平）────────────────

    /** @brief 配置按键引脚（SWITCH_NONE = 禁用，默认禁用） */
    void setSwitchPin(uint8_t pin);
    /** @brief 消抖时间 ms（默认 50） */
    void setKeyPressTime(uint16_t ms) { _keyPressTime = ms; }
    uint16_t getKeyPressTime() const { return _keyPressTime; }
    /** @brief 消抖后的按键状态；未配置返回 false */
    bool isKeyPressed();

private:

    PCF8574& _pcf;
    uint8_t  _index;      // 编码器编号 0..3
    uint8_t  _pinA;       // A 相引脚 = 2*index
    uint8_t  _pinB;       // B 相引脚 = 2*index+1
    uint8_t  _lastPos;    // 上次 2 位状态
    int32_t  _value;      // 计数值

    // ── 按键消抖状态 ──
    uint8_t  _switchPin;      // SWITCH_NONE = 禁用
    uint16_t _keyPressTime;   // 消抖 ms
    uint8_t  _keyStable;      // 当前稳定电平（0=按下 1=松开）
    uint32_t _keyChangeTick;  // 上次电平变化时刻（HAL_GetTick）
};

#endif
