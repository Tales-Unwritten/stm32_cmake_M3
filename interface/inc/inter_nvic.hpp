#pragma once
// ============================================================
// @platform STM32F103xx（STM32F10xxx / Cortex-M3，基于 STM32F1xx HAL 库）
//
// NVIC 统一门面：把全工程的 HAL_NVIC_* / SysTick / PRIMASK 调用收敛到一处。
// 抽象基类 nvic_bus 与具体实现分离，便于移植到其它 Cortex-M 平台。
//
// 关键约定：
//   - 键类型为 IRQn_Type（设备向量号，STM32 / GD32 等 CMSIS 平台同名同义）。
//     外设 → 向量号的映射留在各外设层（inter_usart / inter_exti / dma_channel ...），
//     本层只负责对给定向量做统一、带校验、分组感知的配置。
//   - 无效向量一律返回 false。不再使用 (IRQn_Type)-1 当哨兵——它恰好等于
//     SysTick_IRQn，会让 set_priority 静默改掉系统节拍优先级。
//   - 优先级范围随当前分组变化：默认 NVIC_PRIORITYGROUP_4 → max_preempt=15、max_sub=0。
//
// 实现见 inter_nvic.cpp；本文件只保留接口与声明。
// ============================================================

#ifdef __cplusplus

#include "stm32f1xx_hal.h"
#include <cstdint>

/** @brief 优先级分组（逻辑编号 0~4，与 HAL 的 NVIC_PRIORITYGROUP_x 一一对应） */
enum class priority_group : uint8_t
{
    group0 = 0, ///< 0 位抢占 / 4 位子优先级
    group1 = 1, ///< 1 / 3
    group2 = 2, ///< 2 / 2
    group3 = 3, ///< 3 / 1
    group4 = 4, ///< 4 / 0（HAL_Init 默认）
};

/** @brief SysTick 时钟源（对应 HAL 的 SYSTICK_CLKSOURCE_*） */
enum class systick_clk : uint8_t
{
    hclk_div8 = 0, ///< AHB / 8
    hclk = 1,      ///< AHB
};

/**
 * @brief NVIC 抽象接口
 *
 * 只有一个实现（nvic_ctrl）时仍保留基类，是为了让上层只依赖 nvic_bus&，
 * 移植到其它平台时替换实现即可（与 inter_spi_bus 的思路一致）。
 */
class nvic_bus
{
  public:
    virtual ~nvic_bus() = default;

    // ===== 核心：使能 / 优先级 =====
    virtual bool enable(IRQn_Type irq) = 0;
    virtual bool disable(IRQn_Type irq) = 0;
    virtual bool set_priority(IRQn_Type irq, uint32_t preempt, uint32_t sub = 0) = 0;
    virtual bool get_priority(IRQn_Type irq, uint32_t &preempt, uint32_t &sub) const = 0;

    // ===== Pending / Active =====
    virtual bool get_pending(IRQn_Type irq) const = 0;
    virtual void set_pending(IRQn_Type irq) = 0;
    virtual void clear_pending(IRQn_Type irq) = 0;
    virtual bool get_active(IRQn_Type irq) const = 0;

    // ===== 优先级分组 =====
    virtual void set_priority_grouping(priority_group g) = 0;
    virtual priority_group get_priority_grouping() const = 0;

    // ===== 能力 / 范围查询（随当前分组变化） =====
    virtual uint32_t priority_bits() const = 0;        ///< 芯片实现的优先级位数（F1 = 4）
    virtual uint32_t max_preempt_priority() const = 0; ///< 当前分组下最大抢占优先级
    virtual uint32_t max_sub_priority() const = 0;     ///< 当前分组下最大子优先级
    virtual bool supports_grouping() const = 0;
    virtual bool supports_active() const = 0;

    // ===== 系统 =====
    virtual void system_reset() = 0;

    // ===== SysTick =====
    virtual bool systick_config(uint32_t ticks, systick_clk clk = systick_clk::hclk_div8) = 0;
    virtual void systick_irq_handler() = 0;

    // ===== 全局中断屏蔽（PRIMASK） =====
    virtual uint32_t disable_global_irq() = 0; ///< 返回进入前的 PRIMASK（供恢复）
    virtual void enable_global_irq() = 0;
    virtual uint32_t get_global_irq_state() const = 0;
    virtual void set_global_irq_state(uint32_t state) = 0;
};

/** @brief nvic_bus 的 STM32F1（Cortex-M3）实现 */
class nvic_ctrl final : public nvic_bus
{
  public:
    nvic_ctrl() = default;

    bool enable(IRQn_Type irq) override;
    bool disable(IRQn_Type irq) override;
    bool set_priority(IRQn_Type irq, uint32_t preempt, uint32_t sub) override;
    bool get_priority(IRQn_Type irq, uint32_t &preempt, uint32_t &sub) const override;

    bool get_pending(IRQn_Type irq) const override;
    void set_pending(IRQn_Type irq) override;
    void clear_pending(IRQn_Type irq) override;
    bool get_active(IRQn_Type irq) const override;

    void set_priority_grouping(priority_group g) override;
    priority_group get_priority_grouping() const override;

    uint32_t priority_bits() const override;
    uint32_t max_preempt_priority() const override;
    uint32_t max_sub_priority() const override;
    bool supports_grouping() const override;
    bool supports_active() const override;

    void system_reset() override;

    bool systick_config(uint32_t ticks, systick_clk clk) override;
    void systick_irq_handler() override;

    uint32_t disable_global_irq() override;
    void enable_global_irq() override;
    uint32_t get_global_irq_state() const override;
    void set_global_irq_state(uint32_t state) override;

  private:
    // 私有辅助（声明在此，实现在 inter_nvic.cpp；与工程其它 hpp 风格一致）
    static bool _is_device_irq(IRQn_Type irq) noexcept;   ///< 外部中断：0 .. k_max_irqn
    static bool _is_valid_vector(IRQn_Type irq) noexcept; ///< 含系统异常：-14 .. k_max_irqn
    uint32_t _preempt_bits() const noexcept;              ///< 当前分组下抢占位数
    uint32_t _sub_bits() const noexcept;                  ///< 当前分组下子优先级位数
    static uint32_t _group_to_hal(priority_group g) noexcept;
    static priority_group _hal_to_group(uint32_t raw) noexcept;
};

/** @brief 全局 NVIC 门面（实例在 cpp 内以函数局部静态构造，零堆分配） */
nvic_bus &nvic();

#endif /* __cplusplus */
