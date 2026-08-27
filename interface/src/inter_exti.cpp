// ============================================================
// @platform GD32F4xx（当前平台）
// ============================================================

#include "inter_exti.hpp"

#include "gd32f4xx.h"
#include "gd32f4xx_rcu.h"
#include "gd32f4xx_exti.h"
#include "gd32f4xx_syscfg.h"
#include "gd32f4xx_misc.h"

// ════════════════════════════════════════════════════════════
//  全局端口映射表（EXTI 线 0~22）
//  线 17（RTC 闹钟）由 inter_rtc 管理，槽位保留 nullptr
// ════════════════════════════════════════════════════════════

namespace {
constexpr uint8_t EXTI_LINES = 23;
exti_port *g_exti_map[EXTI_LINES] = {};
}

void _exti_fire(exti_port *p)
{
    if (!p || !p->is_initialized()) return;

    // 检查并清除中断标志
    exti_line_enum line = (exti_line_enum)((uint32_t)EXTI_0 << p->_line);
    if (exti_interrupt_flag_get(line) == SET) {
        exti_interrupt_flag_clear(line);

        if (p->_cb) {
            p->_cb(p->_cb_data);
        }
    }
}

// ════════════════════════════════════════════════════════════
//  构造 / 析构
// ════════════════════════════════════════════════════════════

exti_port::exti_port(const ExtiConfig &cfg)
    : _pin(cfg.port, cfg.pin)
    , _cfg(cfg)
    , _cb(nullptr), _cb_data(nullptr)
    , _initialized(false)
    , _line(0)
{}

exti_port::~exti_port() { deinit(); }

// ════════════════════════════════════════════════════════════
//  线号解析
// ════════════════════════════════════════════════════════════

uint8_t exti_port::_resolve_line() const
{
    if (_cfg.line_type == exti_line_id::gpio)
    {
        if (_cfg.pin == pin_none) return 0xFF;   // 无效引脚

        // pin 是一个 bit，找到它是第几位（0~15）
        uint32_t p = _cfg.pin;
        uint8_t n = 0;
        while (p > 1) { p >>= 1; n++; }
        return n;
    }
    // 系统线：线号 = 枚举值（16~22）
    return static_cast<uint8_t>(_cfg.line_type);
}

// ════════════════════════════════════════════════════════════
//  SYSCFG 端口选择（仅 GPIO 线）
// ════════════════════════════════════════════════════════════

void exti_port::_config_syscfg()
{
    rcu_periph_clock_enable(RCU_SYSCFG);

    // 端口 → EXTI_SOURCE_GPIOx
    uint8_t port_src = 0;
    if      (_cfg.port == GPIOA) port_src = EXTI_SOURCE_GPIOA;
    else if (_cfg.port == GPIOB) port_src = EXTI_SOURCE_GPIOB;
    else if (_cfg.port == GPIOC) port_src = EXTI_SOURCE_GPIOC;
    else if (_cfg.port == GPIOD) port_src = EXTI_SOURCE_GPIOD;
    else if (_cfg.port == GPIOE) port_src = EXTI_SOURCE_GPIOE;
    else if (_cfg.port == GPIOF) port_src = EXTI_SOURCE_GPIOF;
    else if (_cfg.port == GPIOG) port_src = EXTI_SOURCE_GPIOG;
    else if (_cfg.port == GPIOH) port_src = EXTI_SOURCE_GPIOH;
    else if (_cfg.port == GPIOI) port_src = EXTI_SOURCE_GPIOI;
    else { assert(0); return; }   // 非法端口

    uint8_t pin_src = _line;  // EXTI_SOURCE_PIN0 ~ PIN15 = 0~15
    syscfg_exti_line_config(port_src, pin_src);
}

// ════════════════════════════════════════════════════════════
//  IRQ 映射（23 线 → NVIC 通道）
// ════════════════════════════════════════════════════════════

IRQn_Type exti_port::_get_irq() const
{
    if (_line <= 4)  return (IRQn_Type)(EXTI0_IRQn + _line);
    if (_line <= 9)  return EXTI5_9_IRQn;
    if (_line <= 15) return EXTI10_15_IRQn;

    switch (_line)
    {
        case 16: return LVD_IRQn;            // 低电压检测
        // 17: RTC_Alarm_IRQn（inter_rtc 管理，本类不注册）
        case 18: return USBFS_WKUP_IRQn;     // USB FS 唤醒
        case 19: return ENET_WKUP_IRQn;      // 以太网唤醒
        case 20: return USBHS_WKUP_IRQn;     // USB HS 唤醒
        case 21: return TAMPER_STAMP_IRQn;   // RTC 时间戳
        case 22: return RTC_WKUP_IRQn;       // RTC 唤醒
        default: return EXTI0_IRQn;
    }
}

// ════════════════════════════════════════════════════════════
//  ISR 注册
// ════════════════════════════════════════════════════════════

void exti_port::_register_isr()   { g_exti_map[_line] = this; }
void exti_port::_unregister_isr() { g_exti_map[_line] = nullptr; }

// ════════════════════════════════════════════════════════════
//  init / deinit
// ════════════════════════════════════════════════════════════

bool exti_port::init()
{
    if (_initialized) return true;

    _line = _resolve_line();
    if (_line > 22) return false;
    if (_line == 17) return false;                       // RTC 闹钟线由 inter_rtc 管理
    if (g_exti_map[_line] != nullptr) return false;      // ⚠️ 线路互斥：同线已有实例

    if (_cfg.line_type == exti_line_id::gpio)
    {
        _pin.init(mode_input, _cfg.pull);
        _config_syscfg();
    }

    // EXTI 触发配置（系统线硬件固定上升沿）
    exti_line_enum     line = (exti_line_enum)((uint32_t)EXTI_0 << _line);
    exti_trig_type_enum trig = EXTI_TRIG_RISING;
    if (_cfg.line_type == exti_line_id::gpio)
    {
        switch (_cfg.trigger) {
            case exti_trigger::rising:  trig = EXTI_TRIG_RISING;  break;
            case exti_trigger::falling: trig = EXTI_TRIG_FALLING; break;
            default:                     trig = EXTI_TRIG_BOTH;    break;
        }
    }

    exti_init(line, EXTI_INTERRUPT, trig);
    exti_interrupt_flag_clear(line);
    exti_interrupt_enable(line);

    nvic_irq_enable(_get_irq(), _cfg.priority, 0);
    _register_isr();
    _initialized = true;
    return true;
}

void exti_port::deinit()
{
    if (!_initialized) return;

    exti_line_enum line = (exti_line_enum)((uint32_t)EXTI_0 << _line);
    exti_interrupt_disable(line);
    nvic_irq_disable(_get_irq());
    _unregister_isr();

    if (_cfg.line_type == exti_line_id::gpio)
        _pin.deinit();

    _cb = nullptr;
    _initialized = false;
}

// ════════════════════════════════════════════════════════════
//  回调 + 使能 + 软件触发
// ════════════════════════════════════════════════════════════

void exti_port::on_interrupt(callback_t cb, void *data)
{
    // 关中断保护：防止 ISR 在两次写入之间触发，拿到不一致的 cb + data
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    _cb = cb;
    _cb_data = data;
    __set_PRIMASK(mask);
}

void exti_port::enable()  { if (_initialized) nvic_irq_enable(_get_irq(), _cfg.priority, 0); }
void exti_port::disable() { if (_initialized) nvic_irq_disable(_get_irq()); }

void exti_port::software_trigger()
{
    if (!_initialized) return;
    exti_software_interrupt_enable((exti_line_enum)((uint32_t)EXTI_0 << _line));
}

// ════════════════════════════════════════════════════════════
//  ISR 入口（全部内聚在此）
//  GPIO 线 0~15 + 系统线 16/18~22（线 17 由 inter_rtc 管理）
// ════════════════════════════════════════════════════════════

extern "C" void EXTI0_IRQHandler(void)  { _exti_fire(g_exti_map[0]); }
extern "C" void EXTI1_IRQHandler(void)  { _exti_fire(g_exti_map[1]); }
extern "C" void EXTI2_IRQHandler(void)  { _exti_fire(g_exti_map[2]); }
extern "C" void EXTI3_IRQHandler(void)  { _exti_fire(g_exti_map[3]); }
extern "C" void EXTI4_IRQHandler(void)  { _exti_fire(g_exti_map[4]); }

extern "C" void EXTI5_9_IRQHandler(void) {
    for (uint8_t i = 5; i <= 9; i++) _exti_fire(g_exti_map[i]);
}

extern "C" void EXTI10_15_IRQHandler(void) {
    for (uint8_t i = 10; i <= 15; i++) _exti_fire(g_exti_map[i]);
}

/* ── 系统事件线（对齐参考实现 exti_bsp.c） ── */

extern "C" void LVD_IRQHandler(void)          { _exti_fire(g_exti_map[16]); }
extern "C" void USBFS_WKUP_IRQHandler(void)   { _exti_fire(g_exti_map[18]); }
extern "C" void ENET_WKUP_IRQHandler(void)    { _exti_fire(g_exti_map[19]); }
extern "C" void USBHS_WKUP_IRQHandler(void)   { _exti_fire(g_exti_map[20]); }
extern "C" void TAMPER_STAMP_IRQHandler(void) { _exti_fire(g_exti_map[21]); }
extern "C" void RTC_WKUP_IRQHandler(void)     { _exti_fire(g_exti_map[22]); }
