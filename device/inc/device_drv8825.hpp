#pragma once

#ifdef __cplusplus

#include "inter_io_ctrl.hpp"
#include <cstdint>

// ============================================================
//  DRV8825 步进电机驱动
// ============================================================
//  来源：Rob Tillaart 的 Arduino 库 "DRV8825"
//  版本：0.2.2（2026-04-12）
//    URL：https://github.com/RobTillaart/DRV8825
//
//  移植说明（对照参考库 0.2.2）：
//    - 参考库用 uint8_t 引脚号 + pinMode/digitalWrite；本驱动用
//      io_ctrl&（STEP/DIR/EN 必选）+ 可选指针（RST/SLP/M0/M1/M2，
//      nullptr = 未接），init() 对应参考库 begin()
//    - 新增 setMicrostep()：参考库 0.2.2 无微步功能（M0~M2 由用户
//      直接控制）；本驱动按任务要求用 M0/M1/M2 实现数据手册真值表
//      （0b101/110/111 均为 1/32 步）
//    - step() 用 delay_us 产生脉冲（参考库 stepPulseLength，默认
//      2µs，数据手册要求 ≥ 1.9µs）
//    - 方向常量沿用参考库命名（DRV8825_CLOCK_WISE 等）
//    - 无浮点、无堆分配、无异常
//
//  硬件注意：
//    - EN/RST/SLP 均低有效；RST/SLP 芯片内部有下拉（默认复位/休眠
//      态），建议接上由驱动控制；未接时需外部上拉
//    - M0~M2 未接时需外部拉低以保持全步进（模块板载情况以实物为准）
// ============================================================

class DRV8825
{
public:

    // ── 方向（与参考库一致）───────────────────────────────

    static constexpr uint8_t CLOCK_WISE        = 0;   // DIR = LOW
    static constexpr uint8_t COUNTERCLOCK_WISE = 1;   // DIR = HIGH

    // ── 微步模式（M2 M1 M0 组合，数据手册真值表）──────────

    enum class Microstep : uint8_t
    {
        Full         = 0b000,   // 全步
        Half         = 0b001,   // 1/2
        Quarter      = 0b010,   // 1/4
        Eighth       = 0b011,   // 1/8
        Sixteenth    = 0b100,   // 1/16
        ThirtySecond = 0b101,   // 1/32（0b110/0b111 同）
    };

    /**
     * @brief 构造：STEP/DIR/EN 必选；RST/SLP/M0/M1/M2 可选（nullptr）
     * @note  引脚由外部 init()（io_ctrl），本类构造只保存引用
     */
    DRV8825(io_ctrl& step, io_ctrl& dir, io_ctrl& en,
            io_ctrl* rst = nullptr, io_ctrl* slp = nullptr,
            io_ctrl* m0 = nullptr, io_ctrl* m1 = nullptr, io_ctrl* m2 = nullptr);

    /** @brief 初始化引脚电平（对应参考库 begin()）：默认使能、全步进 */
    void init();

    // ── 方向 ──────────────────────────────────────────────

    bool    setDirection(uint8_t direction = CLOCK_WISE);  // 越界返回 false
    uint8_t getDirection();                                // 读 DIR 引脚实际电平

    // ── 步进 ──────────────────────────────────────────────

    void     setStepsPerRevolution(uint16_t stepsPerRevolution);
    uint16_t getStepsPerRevolution() const { return _stepsPerRevolution; }
    void     step();                                    // 走一步（当前方向）
    void     stepMotor(uint32_t steps, uint8_t direction);  // 设方向并走 N 步
    uint32_t resetSteps(uint32_t s = 0);                // 清零步数计数，返回旧值
    uint32_t getSteps() const { return _steps; }

    // ── 位置（步数，在 stepsPerRevolution 内循环 0..N-1）──

    bool     setPosition(uint16_t position);            // 越界返回 false
    uint16_t getPosition() const { return _position; }

    // ── 脉冲宽度 ─────────────────────────────────────────

    /** @brief STEP 脉冲高低电平宽度 µs；数据手册要求 ≥ 1.9µs */
    void     setStepPulseLength(uint16_t us = 2);
    uint16_t getStepPulseLength() const { return _stepPulseLength; }
    /** @brief setStepPulseLength 的别名（与任务命名对齐） */
    void     setStepTime(uint16_t us = 2) { setStepPulseLength(us); }

    // ── 使能 / 复位 / 休眠（均低有效）────────────────────

    bool enable();                 // EN = LOW
    bool disable();                // EN = HIGH
    bool isEnabled();              // EN == LOW
    bool reset();                  // RST 拉低 1ms 再拉高（需接 RST）
    bool sleep();                  // SLP = LOW（需接 SLP）
    bool wakeup();                 // SLP = HIGH
    bool isSleeping();             // SLP == LOW

    // ── 微步（需接 M0/M1/M2，未接返回 false）──────────────

    bool setMicrostep(Microstep ms);

private:

    io_ctrl& _step;
    io_ctrl& _dir;
    io_ctrl& _en;
    io_ctrl* _rst;      // 可为 nullptr
    io_ctrl* _slp;      // 可为 nullptr
    io_ctrl* _m0;       // 可为 nullptr
    io_ctrl* _m1;       // 可为 nullptr
    io_ctrl* _m2;       // 可为 nullptr

    uint8_t  _direction;          // CLOCK_WISE / COUNTERCLOCK_WISE
    uint16_t _stepsPerRevolution; // 0 = 不跟踪位置
    uint32_t _steps;              // 累计步数
    uint16_t _position;           // 当前圈内位置
    uint16_t _stepPulseLength;    // µs
};

#endif
