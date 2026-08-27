// ============================================================
// @platform GD32F4xx（当前平台）
//   移植到新 MCU 时，本文件需要完整重写，但公共接口签名不变。
//   需要替换的映射：
//     - 外设基址:   TIMER0~TIMER7 → 目标平台外设基址
//     - 时钟使能:   rcu_periph_clock_enable(RCU_TIMERx)
//     - 外设初始化:  timer_init(timer_parameter_struct)
//     - PWM 配置:   timer_channel_output_config / pulse_value_config
//     - 中断管理:   nvic_irq_enable → 目标平台 NVIC API
//     - ISR 入口:   TIMER1_IRQHandler → 目标平台中断向量名
//     - GPIO 复用:  AF 编号 → 目标平台 AF 编号
// ============================================================

#include "inter_timer.hpp"

#include <new>

#include "gd32f4xx.h"
#include "gd32f4xx_misc.h"
#include "gd32f4xx_rcu.h"
#include "gd32f4xx_timer.h"

// ============================================================
//  全局端口映射表
// ============================================================

namespace
{

constexpr uint8_t TIMER_MAX = 8;
timer_port *g_port_map[TIMER_MAX] = {};

uint8_t _timer_index(timer_id id)
{
    return static_cast<uint8_t>(id);
}

// ── 频率自动计算辅助 ────────────────────────────────────

/** APB1 总线判断（GD32F470: TIMER1~6 在 APB1，TIMER0/7 在 APB2） */
bool _timer_on_apb1(timer_id id)
{
    return id != timer_id::timer0 && id != timer_id::timer7;
}

/** 32 位计数器判断（GD32F470: TIMER1、TIMER4 为 32 位） */
bool _timer_is_32bit(timer_id id)
{
    return id == timer_id::timer1 || id == timer_id::timer4;
}

/**
 * @brief 推导定时器时钟频率
 *
 * GD32F4xx 定时器时钟规则（区别于 STM32 的固定 2 倍 APB）：
 *   TIMERSEL=0：APBxPSC 分频 ≤2（字段值 ≤0b100）→ CK_TIMERx = CK_AHB；否则 = 2×CK_APBx
 *   TIMERSEL=1：APBxPSC 分频 ≤4（字段值 ≤0b101）→ CK_TIMERx = CK_AHB；否则 = 4×CK_APBx
 */
uint32_t _timer_clock_hz(bool apb1)
{
    uint32_t ahb = rcu_clock_freq_get(CK_AHB);
    uint32_t apb = rcu_clock_freq_get(apb1 ? CK_APB1 : CK_APB2);
    uint32_t psc = apb1 ? ((RCU_CFG0 & RCU_CFG0_APB1PSC) >> 10U) : ((RCU_CFG0 & RCU_CFG0_APB2PSC) >> 13U);
    if (RCU_CFG1 & RCU_CFG1_TIMERSEL)
    {
        return (psc <= 5U) ? ahb : 4u * apb;
    }
    return (psc <= 4U) ? ahb : 2u * apb;
}

/**
 * @brief 由目标频率自动计算 prescaler/period（O(1)）
 *
 * 约束：(psc+1) × (period+1) = timer_clk / freq，且 period ≤ 定时器位宽上限。
 * 取最小 psc 使 period 尽量大 → 分辨率最高。
 *
 * @param timer_clk  定时器时钟（Hz）
 * @param freq_hz    目标频率（Hz，>0）
 * @param is_32bit   计数器是否为 32 位
 * @param[out] prescaler 计算结果
 * @param[out] period    计算结果
 */
void _auto_calc(uint32_t timer_clk, uint32_t freq_hz, bool is_32bit, uint32_t &prescaler, uint32_t &period)
{
    // 一个周期内的计数 tick 数（四舍五入）
    uint64_t n = ((uint64_t)timer_clk + (freq_hz / 2U)) / freq_hz;
    if (n < 2U)
        n = 2U; // 目标频率高于时钟时的退化保护

    const uint64_t max_period_p1 = is_32bit ? 0x100000000ULL : 65536ULL;
    uint32_t psc = (n > max_period_p1) ? static_cast<uint32_t>((n - 1U) / max_period_p1) : 0U;
    if (psc > 65535U)
        psc = 65535U;

    uint64_t period_p1 = n / (static_cast<uint64_t>(psc) + 1U);
    if (period_p1 < 2U)
        period_p1 = 2U;

    prescaler = psc;
    period = static_cast<uint32_t>(period_p1 - 1U);
}

} // anonymous namespace

// ============================================================
//  ISR 触发入口（供 ISR 调用）
// ============================================================

void _timer_isr_fire(timer_port *port)
{
    if (!port || !port->is_initialized())
        return;

    // 检查更新中断标志
    if (timer_interrupt_flag_get(port->_periph, TIMER_INT_FLAG_UP) == SET)
    {
        timer_interrupt_flag_clear(port->_periph, TIMER_INT_FLAG_UP);

        if (port->_timeout_cb)
        {
            port->_timeout_cb(port->_timeout_cb_data);
        }
    }
}

// ============================================================
//  构造 / 析构
// ============================================================

timer_port::timer_port(const TimerPortConfig &cfg)
    : _cfg(cfg), _ch_pins{nullptr, nullptr, nullptr, nullptr}, _initialized(false), _periph(0), _timeout_cb(nullptr),
      _timeout_cb_data(nullptr)
{
    // placement new: 在 aligned storage 上构造实际使用的通道
    // [FIX] 限制 channel_count 最大为 4，防止数组越界
    uint8_t count = (cfg.channel_count > 4) ? 4 : cfg.channel_count;
    for (uint8_t i = 0; i < count; i++)
    {
        if (cfg.channels[i].port != nullptr)
        {
            _ch_pins[i] = new (_ch_storage[i].data) io_ctrl(cfg.channels[i].port, cfg.channels[i].pin);
        }
    }
}

timer_port::~timer_port()
{
    deinit();
}

// ============================================================
//  时钟使能
// ============================================================

void timer_port::_enable_clock()
{
    // [PORT] GD32 时钟映射
    switch (_cfg.periph)
    {
    case timer_id::timer0:
        rcu_periph_clock_enable(RCU_TIMER0);
        break;
    case timer_id::timer1:
        rcu_periph_clock_enable(RCU_TIMER1);
        break;
    case timer_id::timer2:
        rcu_periph_clock_enable(RCU_TIMER2);
        break;
    case timer_id::timer3:
        rcu_periph_clock_enable(RCU_TIMER3);
        break;
    case timer_id::timer4:
        rcu_periph_clock_enable(RCU_TIMER4);
        break;
    case timer_id::timer5:
        rcu_periph_clock_enable(RCU_TIMER5);
        break;
    case timer_id::timer6:
        rcu_periph_clock_enable(RCU_TIMER6);
        break;
    case timer_id::timer7:
        rcu_periph_clock_enable(RCU_TIMER7);
        break;
    }
}

// ============================================================
//  IRQ 映射
// ============================================================

IRQn_Type timer_port::_get_irq() const
{
    // [PORT] GD32 中断向量映射
    // TIMER0/TIMER7 共享中断线，需在 ISR 中判断来源
    switch (_cfg.periph)
    {
    case timer_id::timer0:
        return TIMER0_UP_TIMER9_IRQn;
    case timer_id::timer1:
        return TIMER1_IRQn;
    case timer_id::timer2:
        return TIMER2_IRQn;
    case timer_id::timer3:
        return TIMER3_IRQn;
    case timer_id::timer4:
        return TIMER4_IRQn;
        // GD32F4xx: TIMER5 与 DAC0/DAC1 共享中断线 TIMER5_DAC_IRQn
        // [PORT] 若目标芯片有独立 TIMER5 中断向量，改用之
#ifdef TIMER5_DAC_IRQn
    case timer_id::timer5:
        return TIMER5_DAC_IRQn;
#else
    case timer_id::timer5:
        return TIMER6_IRQn; // 降级到 TIMER6
#endif
    case timer_id::timer6:
        return TIMER6_IRQn;
    case timer_id::timer7:
        return TIMER7_UP_TIMER12_IRQn;
    }
    return TIMER2_IRQn;
}

// ============================================================
//  ISR 注册 / 注销
// ============================================================

void timer_port::_register_isr()
{
    uint8_t idx = _timer_index(_cfg.periph);
    if (idx < TIMER_MAX)
        g_port_map[idx] = this;
}

void timer_port::_unregister_isr()
{
    uint8_t idx = _timer_index(_cfg.periph);
    if (idx < TIMER_MAX)
        g_port_map[idx] = nullptr;
}

// ============================================================
//  init
// ============================================================

void timer_port::init()
{
    if (_initialized)
        return;

    // 外设基址映射
    // [PORT] GD32 外设基址
    switch (_cfg.periph)
    {
    case timer_id::timer0:
        _periph = TIMER0;
        break;
    case timer_id::timer1:
        _periph = TIMER1;
        break;
    case timer_id::timer2:
        _periph = TIMER2;
        break;
    case timer_id::timer3:
        _periph = TIMER3;
        break;
    case timer_id::timer4:
        _periph = TIMER4;
        break;
    case timer_id::timer5:
        _periph = TIMER5;
        break;
    case timer_id::timer6:
        _periph = TIMER6;
        break;
    case timer_id::timer7:
        _periph = TIMER7;
        break;
    }

    _enable_clock();

    // 频率自动计算：frequency_hz > 0 时推导 prescaler/period 并回写配置
    if (_cfg.frequency_hz > 0)
    {
        _auto_calc(_timer_clock_hz(_timer_on_apb1(_cfg.periph)), _cfg.frequency_hz, _timer_is_32bit(_cfg.periph),
                   _cfg.prescaler, _cfg.period);
    }

    // 定时器基本配置
    timer_parameter_struct timer_cfg;
    timer_struct_para_init(&timer_cfg);

    timer_cfg.prescaler = _cfg.prescaler;
    timer_cfg.period = _cfg.period;
    timer_cfg.clockdivision = TIMER_CKDIV_DIV1;
    timer_cfg.counterdirection = TIMER_COUNTER_UP;
    timer_cfg.repetitioncounter = 0;

    timer_init(_periph, &timer_cfg);

    // PWM 通道配置
    // GPIO AF 初始化
    for (uint8_t i = 0; i < _cfg.channel_count && i < 4; i++)
    {
        if (_cfg.channels[i].port == nullptr)
            continue;

        // deinit 后重建：_ch_pins[i] 已被销毁为 nullptr
        if (_ch_pins[i] == nullptr)
        {
            _ch_pins[i] = new (_ch_storage[i].data) io_ctrl(_cfg.channels[i].port, _cfg.channels[i].pin);
        }

        _ch_pins[i]->init(mode_af_pp, nopull, speed_high);
        _ch_pins[i]->set_af(_cfg.channels[i].af);

        // PWM 输出配置
        timer_oc_parameter_struct oc_cfg;
        timer_channel_output_struct_para_init(&oc_cfg);
        oc_cfg.outputstate = TIMER_CCX_ENABLE;
        oc_cfg.ocpolarity = TIMER_OC_POLARITY_HIGH;
        timer_channel_output_config(_periph, (uint16_t)i, &oc_cfg);
        timer_channel_output_pulse_value_config(_periph, (uint16_t)i, 0);
        timer_channel_output_mode_config(_periph, (uint16_t)i, TIMER_OC_MODE_PWM0);
    }

    // 高级定时器主输出使能（CCHP.POEN，即 MOE）。默认关闭，不使能则
    // timer0/timer7 的 PWM 引脚无输出。通用定时器无 CCHP 寄存器，不可调用。
    if (_cfg.periph == timer_id::timer0 || _cfg.periph == timer_id::timer7)
    {
        timer_primary_output_config(_periph, ENABLE);
    }

    // 清除残留中断标志，再使能中断
    timer_interrupt_flag_clear(_periph, TIMER_INT_FLAG_UP);
    timer_interrupt_enable(_periph, TIMER_INT_UP);
    nvic_irq_enable(_get_irq(), 0, 0);

    _register_isr();
    _initialized = true;
}

// ============================================================
//  deinit
// ============================================================

void timer_port::deinit()
{
    if (!_initialized)
        return;

    timer_interrupt_disable(_periph, TIMER_INT_UP);
    nvic_irq_disable(_get_irq());
    _unregister_isr();

    timer_disable(_periph);

    for (uint8_t i = 0; i < _cfg.channel_count && i < 4; i++)
    {
        if (_ch_pins[i])
        {
            _ch_pins[i]->deinit();
            _ch_pins[i]->~io_ctrl();
            _ch_pins[i] = nullptr;
        }
    }

    _timeout_cb = nullptr;
    _periph = 0;
    _initialized = false;
}

// ============================================================
//  基本定时
// ============================================================

void timer_port::start()
{
    if (!_initialized)
        return;
    timer_enable(_periph);
}

void timer_port::stop()
{
    if (!_initialized)
        return;
    timer_disable(_periph);
}

uint32_t timer_port::counter() const
{
    if (!_initialized)
        return 0;
    return timer_counter_read(_periph);
}

void timer_port::set_period(uint32_t period)
{
    if (!_initialized)
        return;
    timer_autoreload_value_config(_periph, period);
}

// ============================================================
//  PWM
// ============================================================

void timer_port::pwm_set_duty(uint8_t channel, uint32_t value)
{
    if (!_initialized || channel >= _cfg.channel_count)
        return;
    timer_channel_output_pulse_value_config(_periph, (uint16_t)channel, value);
}

void timer_port::pwm_set_duty_percent(uint8_t channel, float percent)
{
    if (!_initialized || channel >= _cfg.channel_count)
        return;

    if (percent <= 0.0f)
    {
        timer_channel_output_pulse_value_config(_periph, (uint16_t)channel, 0);
    }
    else if (percent >= 100.0f)
    {
        timer_channel_output_pulse_value_config(_periph, (uint16_t)channel, _cfg.period);
    }
    else
    {
        uint32_t value = static_cast<uint32_t>(_cfg.period * percent / 100.0f);
        timer_channel_output_pulse_value_config(_periph, (uint16_t)channel, value);
    }
}

// ============================================================
//  回调
// ============================================================

void timer_port::on_timeout(timeout_cb_t cb, void *user_data)
{
    _timeout_cb = cb;
    _timeout_cb_data = user_data;
}

// ============================================================
//  ISR 入口
// ============================================================

extern "C" void TIMER0_UP_TIMER9_IRQHandler(void)
{
    // TIMER0 和 TIMER9 共享中断线。当前仅处理 TIMER0。
    _timer_isr_fire(g_port_map[_timer_index(timer_id::timer0)]);
}

extern "C" void TIMER1_IRQHandler(void)
{
    _timer_isr_fire(g_port_map[_timer_index(timer_id::timer1)]);
}

extern "C" void TIMER2_IRQHandler(void)
{
    _timer_isr_fire(g_port_map[_timer_index(timer_id::timer2)]);
}

extern "C" void TIMER3_IRQHandler(void)
{
    _timer_isr_fire(g_port_map[_timer_index(timer_id::timer3)]);
}

extern "C" void TIMER4_IRQHandler(void)
{
    _timer_isr_fire(g_port_map[_timer_index(timer_id::timer4)]);
}

// TIMER5 与 DAC 共享中断线（GD32F4xx）：DAC 欠载等事件也会进入本 ISR，
// _timer_isr_fire 内通过 UP 标志判断，非定时器事件直接返回，无副作用。
#ifdef TIMER5_DAC_IRQn
extern "C" void TIMER5_DAC_IRQHandler(void)
{
    _timer_isr_fire(g_port_map[_timer_index(timer_id::timer5)]);
}
#endif

extern "C" void TIMER6_IRQHandler(void)
{
    _timer_isr_fire(g_port_map[_timer_index(timer_id::timer6)]);
}

extern "C" void TIMER7_UP_TIMER12_IRQHandler(void)
{
    _timer_isr_fire(g_port_map[_timer_index(timer_id::timer7)]);
}
