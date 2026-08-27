#pragma once
// ============================================================
// @platform GD32F4xx
//   移植到新 MCU 时，本 .hpp 文件需替换：
//     - [PORT] #include "gd32f4xx.h" → 目标 SDK 头文件
//     - [PORT] enum class timer_id（按目标芯片调整枚举项）
//     - [PORT] timer_parameter_struct → 目标定时器结构体
// ============================================================

#ifdef __cplusplus

#include "gd32f4xx.h" // [PORT] 替换为目标 SDK 头文件
#include "inter_io_ctrl.hpp"
#include <cstdint>

// ============================================================
//  平台中性类型
// ============================================================

/** @brief 定时器外设 ID */
enum class timer_id : uint8_t
{
    timer0,
    timer1,
    timer2,
    timer3,
    timer4,
    timer5,
    timer6,
    timer7
};

// ============================================================
//  配置结构体（POD）
// ============================================================

/** @brief PWM 通道引脚配置 */
struct TimerChannelConfig
{
    GPIO_TypeDef* port = nullptr;  ///< GPIO 端口（nullptr = 不使用此通道）
    pin_enum_t    pin  = pin_none;  ///< 引脚掩码
    afio_enum_t   af   = afio_enum_t::NONE;  ///< STM32F1 AFIO 重映射选项
};

/**
 * @brief 定时器端口配置
 *
 * 定时器时钟 = APB总线时钟 / (prescaler + 1)
 * 溢出周期   = (period + 1) / f_timer
 *
 * PWM 频率 = f_timer / (period + 1)
 * PWM 占空比 = pwm_set_duty(ch, val) 中 val 范围 0 ~ period
 */
struct TimerPortConfig
{
    timer_id periph; ///< 外设 ID
    /**
     * @brief 目标频率（Hz），支持两种配置方式：
     *   > 0：自动计算 prescaler/period（init() 时回写），
     *        定时器时钟由 rcu_clock_freq_get + TIMERSEL 位自动推导；
     *   == 0：手动指定 prescaler/period。
     */
    uint32_t frequency_hz;
    uint32_t prescaler;             ///< 预分频值（自动模式：init 回写计算结果）
    uint32_t period;                ///< 自动重装载值（自动模式：init 回写计算结果）
    TimerChannelConfig channels[4]; ///< PWM 通道（port=0 表示不用）
    uint8_t channel_count;          ///< 实际使用的 PWM 通道数
};

// ============================================================
//  定时器端口（基本定时 + PWM 输出）
// ============================================================

/**
 * @brief 定时器端口（基本定时 + PWM 输出 + 溢出中断回调）
 *
 * 使用示例（1kHz 定时中断，自动计算分频）：
 *   static timer_port tick_timer({
 *       timer_id::timer2, 1000, 0, 0, {}, 0
 *   });
 *   tick_timer.init();
 *   tick_timer.on_timeout([](void*) { led.toggle(); });
 *   tick_timer.start();
 *
 * 使用示例（PWM 输出，1kHz, 50% 占空比，手动指定分频）：
 *   static timer_port pwm({
 *       timer_id::timer2, 0, 119, 999,
 *       {{{GPIOA, pin1, afio_enum_t::NONE}}}, 1
 *   });
 *   pwm.init();
 *   pwm.pwm_set_duty_percent(0, 50.0f);  // 50% of period
 *   pwm.start();
 */
class timer_port
{
  public:
    explicit timer_port(const TimerPortConfig &cfg);
    ~timer_port();

    timer_port(const timer_port &) = delete;
    timer_port &operator=(const timer_port &) = delete;

    // ── 生命周期 ──────────────────────────────────────────
    void init();
    void deinit();
    [[nodiscard]] bool is_initialized() const noexcept
    {
        return _initialized;
    }

    // ── 基本定时 ──────────────────────────────────────────

    void start();
    void stop();
    [[nodiscard]] uint32_t counter() const;
    void set_period(uint32_t period);

    // ── PWM ───────────────────────────────────────────────

    /** @brief 设置 PWM 占空比
     *  @param channel 通道号 0~3
     *  @param value   占空比值，范围 0 ~ period
     */
    void pwm_set_duty(uint8_t channel, uint32_t value);

    /** @brief 设置 PWM 占空比（百分比）
     *  @param channel 通道号 0~3
     *  @param percent 占空比 0.0~100.0（基于当前 period）
     */
    void pwm_set_duty_percent(uint8_t channel, float percent);

    // ── 溢出回调（ISR 上下文调用，必须短） ────────────────

    using timeout_cb_t = void (*)(void *user_data);
    void on_timeout(timeout_cb_t cb, void *user_data = nullptr);

    // ── 查询 ──────────────────────────────────────────────

    [[nodiscard]] timer_id periph() const noexcept
    {
        return _cfg.periph;
    }

  private:
    void _enable_clock();
    IRQn_Type _get_irq() const;
    void _register_isr();
    void _unregister_isr();

    // ISR 可访问（friend 声明在 .cpp）
    friend void _timer_isr_fire(timer_port *port);

    TimerPortConfig _cfg;

    // PWM 通道引脚存储（placement new，零堆分配）
    // io_ctrl 没有默认构造函数，使用 aligned storage
    struct
    {
        alignas(io_ctrl) uint8_t data[sizeof(io_ctrl)];
    } _ch_storage[4];
    io_ctrl *_ch_pins[4]; // 指向 _ch_storage 的有效指针

    bool _initialized;
    uint32_t _periph;

    // 回调
    timeout_cb_t _timeout_cb;
    void *_timeout_cb_data;
};

#endif /* __cplusplus */
