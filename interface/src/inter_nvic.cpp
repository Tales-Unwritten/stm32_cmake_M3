// ============================================================
// @platform STM32F103xx（Cortex-M3 / STM32F1xx HAL）
//
// nvic_bus 的 STM32F1 实现。设计约定见 inter_nvic.hpp。
//
// 与"直接转发 HAL"的差异：
//   1. 无效向量拦截：非本芯片定义的外部中断一律返回 false。不再用
//      (IRQn_Type)-1 当哨兵——它等于 SysTick_IRQn，会让 set_priority 走
//      __NVIC_SetPriority 的系统异常分支、静默改掉 SysTick 优先级。
//   2. 分组感知：set_priority 先按当前分组算有效位宽再夹取，而不是恒定 15；
//      max_preempt_priority / max_sub_priority 也随分组变化。
//   3. get_priority_grouping 修正：HAL_NVIC_GetPriorityGrouping() 返回的是原始
//      PRIGROUP（Group4 → 3），需换算回逻辑分组（7 - PRIGROUP）。
// ============================================================

#include "inter_nvic.hpp"

namespace
{
// F103xE 最大外部中断向量号（DMA2_Channel4_5_IRQn = 59）。
// 移植到其它 F1 容量/平台时改这里即可。
constexpr IRQn_Type k_max_irqn = DMA2_Channel4_5_IRQn;

// 逻辑分组 0~4 → HAL 的 PRIGROUP 编码（NVIC_PRIORITYGROUP_x）
constexpr uint32_t k_group_to_hal[5] = {
    NVIC_PRIORITYGROUP_0, NVIC_PRIORITYGROUP_1, NVIC_PRIORITYGROUP_2, NVIC_PRIORITYGROUP_3, NVIC_PRIORITYGROUP_4,
};
} // namespace

// ── 私有辅助 ─────────────────────────────────────────────────

bool nvic_ctrl::_is_device_irq(IRQn_Type irq) noexcept
{
    return (static_cast<int32_t>(irq) >= 0) && (static_cast<int32_t>(irq) <= static_cast<int32_t>(k_max_irqn));
}

bool nvic_ctrl::_is_valid_vector(IRQn_Type irq) noexcept
{
    // 系统异常（SysTick/PendSV/...，-14..-1）也可读写优先级
    return (static_cast<int32_t>(irq) >= static_cast<int32_t>(NonMaskableInt_IRQn)) &&
           (static_cast<int32_t>(irq) <= static_cast<int32_t>(k_max_irqn));
}

uint32_t nvic_ctrl::_group_to_hal(priority_group g) noexcept
{
    const uint32_t idx = static_cast<uint8_t>(g);
    return k_group_to_hal[(idx < 5U) ? idx : 4U];
}

priority_group nvic_ctrl::_hal_to_group(uint32_t raw) noexcept
{
    // 原始 PRIGROUP 与逻辑分组反序：Group0↔7 ... Group4↔3
    int32_t logical = 7 - static_cast<int32_t>(raw & 0x07U);
    if (logical < 0)
        logical = 0;
    if (logical > 4)
        logical = 4;
    return static_cast<priority_group>(logical);
}

uint32_t nvic_ctrl::_preempt_bits() const noexcept
{
    const int32_t raw = static_cast<int32_t>(HAL_NVIC_GetPriorityGrouping() & 0x07U);
    int32_t bits = 7 - raw;
    if (bits < 0)
        bits = 0;
    if (bits > static_cast<int32_t>(__NVIC_PRIO_BITS))
        bits = static_cast<int32_t>(__NVIC_PRIO_BITS);
    return static_cast<uint32_t>(bits);
}

uint32_t nvic_ctrl::_sub_bits() const noexcept
{
    return static_cast<uint32_t>(__NVIC_PRIO_BITS) - _preempt_bits();
}

// ── 核心：使能 / 优先级 ──────────────────────────────────────

bool nvic_ctrl::enable(IRQn_Type irq)
{
    if (!_is_device_irq(irq))
        return false;
    HAL_NVIC_EnableIRQ(irq);
    return true;
}

bool nvic_ctrl::disable(IRQn_Type irq)
{
    if (!_is_device_irq(irq))
        return false;
    HAL_NVIC_DisableIRQ(irq);
    return true;
}

bool nvic_ctrl::set_priority(IRQn_Type irq, uint32_t preempt, uint32_t sub)
{
    if (!_is_valid_vector(irq))
        return false;

    // 按当前分组算有效位宽后夹取（Group4 → max_preempt=15、max_sub=0）
    const uint32_t max_preempt = (1U << _preempt_bits()) - 1U;
    const uint32_t max_sub = (1U << _sub_bits()) - 1U;
    if (preempt > max_preempt)
        preempt = max_preempt;
    if (sub > max_sub)
        sub = max_sub;

    HAL_NVIC_SetPriority(irq, preempt, sub);
    return true;
}

bool nvic_ctrl::get_priority(IRQn_Type irq, uint32_t &preempt, uint32_t &sub) const
{
    if (!_is_valid_vector(irq))
    {
        preempt = 0U;
        sub = 0U;
        return false;
    }
    // HAL_NVIC_GetPriority 的 PriorityGroup 形参期望原始 PRIGROUP
    // （NVIC_DecodePriority 内部会 & 0x07），故直接传 HAL_NVIC_GetPriorityGrouping()
    HAL_NVIC_GetPriority(irq, HAL_NVIC_GetPriorityGrouping(), &preempt, &sub);
    return true;
}

// ── Pending / Active ─────────────────────────────────────────

bool nvic_ctrl::get_pending(IRQn_Type irq) const
{
    return _is_device_irq(irq) && (HAL_NVIC_GetPendingIRQ(irq) != 0U);
}

void nvic_ctrl::set_pending(IRQn_Type irq)
{
    if (_is_device_irq(irq))
        HAL_NVIC_SetPendingIRQ(irq);
}

void nvic_ctrl::clear_pending(IRQn_Type irq)
{
    if (_is_device_irq(irq))
        HAL_NVIC_ClearPendingIRQ(irq);
}

bool nvic_ctrl::get_active(IRQn_Type irq) const
{
    return _is_device_irq(irq) && (HAL_NVIC_GetActive(irq) != 0U);
}

// ── 优先级分组 ───────────────────────────────────────────────

void nvic_ctrl::set_priority_grouping(priority_group g)
{
    HAL_NVIC_SetPriorityGrouping(_group_to_hal(g));
}

priority_group nvic_ctrl::get_priority_grouping() const
{
    return _hal_to_group(HAL_NVIC_GetPriorityGrouping());
}

// ── 能力 / 范围查询 ──────────────────────────────────────────

uint32_t nvic_ctrl::priority_bits() const
{
    return static_cast<uint32_t>(__NVIC_PRIO_BITS);
}

uint32_t nvic_ctrl::max_preempt_priority() const
{
    return (1U << _preempt_bits()) - 1U;
}

uint32_t nvic_ctrl::max_sub_priority() const
{
    return (1U << _sub_bits()) - 1U;
}

bool nvic_ctrl::supports_grouping() const
{
    return true;
}

bool nvic_ctrl::supports_active() const
{
    return true;
}

// ── 系统 ─────────────────────────────────────────────────────

void nvic_ctrl::system_reset()
{
    HAL_NVIC_SystemReset();
}

// ── SysTick ──────────────────────────────────────────────────

bool nvic_ctrl::systick_config(uint32_t ticks, systick_clk clk)
{
    // HAL_SYSTICK_Config 内部 SysTick_Config：强制 CLKSOURCE=HCLK、使能 TICKINT。
    // 失败（ticks 非法/超 24 位）时返回非 0，此时不改时钟源。
    if (HAL_SYSTICK_Config(ticks) != 0U)
        return false;
    HAL_SYSTICK_CLKSourceConfig((clk == systick_clk::hclk) ? SYSTICK_CLKSOURCE_HCLK : SYSTICK_CLKSOURCE_HCLK_DIV8);
    return true;
}

void nvic_ctrl::systick_irq_handler()
{
    HAL_SYSTICK_IRQHandler();
}

// ── 全局中断屏蔽（PRIMASK） ──────────────────────────────────

uint32_t nvic_ctrl::disable_global_irq()
{
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
}

void nvic_ctrl::enable_global_irq()
{
    __enable_irq();
}

uint32_t nvic_ctrl::get_global_irq_state() const
{
    return __get_PRIMASK();
}

void nvic_ctrl::set_global_irq_state(uint32_t state)
{
    __set_PRIMASK(state);
}

// ── 全局访问 ─────────────────────────────────────────────────

nvic_bus &nvic()
{
    // 函数局部静态：避免命名空间级静态对象的初始化顺序问题，零堆分配
    static nvic_ctrl instance;
    return instance;
}
