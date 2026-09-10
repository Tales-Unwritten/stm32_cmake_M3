#pragma once
// ============================================================
// @platform STM32F103xx（STM32F10xxx / Cortex-M3，基于 STM32F1xx HAL 库）
//   从 GD32F4xx 版本移植：线配置走 HAL EXTI 驱动（stm32f1xx_hal_exti.c，
//   含 AFIO 端口选择），NVIC 由 HAL_NVIC_* 管理；
//   ISR 分发表 + 线互斥结构对齐参考实现（王海涛 exti_bsp.c）。
// ============================================================

#include "inter_io_ctrl.hpp" // 引入 stm32f1xx_hal.h 与 GPIO/pin/pull 类型

#ifdef __cplusplus

#include <cstdint>

enum class exti_trigger : uint8_t { rising, falling, both };

/**
 * @brief EXTI 线类型（GPIO 线 0~15 + 系统事件线 16~18）
 *
 * STM32F103xE 资源（F1 无 SYSCFG，GPIO 线经 AFIO->EXTICR 选择端口）：
 *  - GPIO 线：端口 A~E 任意引脚（VET6 无 F/G），init 时配置 AFIO 端口选择
 *  - 系统线：PVD 欠压检测 / RTC 闹钟 / USB 设备唤醒
 *  - ⚠️ rtc_alarm(17) 的 ISR 由 inter_rtc（rtc_port）管理，本类不注册该线
 *  - GD32F4 版的 ethernet/usb_hs/rtc_timestamp/rtc_wakeup（19~22）F103 无
 */
enum class exti_line_id : uint8_t {
    gpio      = 0,   ///< GPIO 引脚（配合 port/pin）
    pvd       = 16,  ///< EXTI16 低电压检测（PVD；检测阈值/使能需另行 HAL_PWR_ConfigPVD + HAL_PWR_EnablePVD）
    rtc_alarm = 17,  ///< EXTI17 RTC 闹钟（⚠️ inter_rtc 管理，本类不支持）
    usb_fs    = 18,  ///< EXTI18 USB 设备唤醒（需 USB 外设挂起机制配合）
};

struct ExtiConfig {
    exti_line_id  line_type = exti_line_id::gpio;
    GPIO_TypeDef* port      = nullptr;   ///< GPIOA~GPIOE（仅 gpio 线）
    pin_enum_t    pin       = pin_none;  ///< GPIO_PIN_0 ~ GPIO_PIN_15（仅 gpio 线）
    exti_trigger  trigger   = exti_trigger::rising;  ///< 触发沿（仅 gpio 线；系统线固定上升沿）
    pull_enum_t   pull      = nopull;    ///< 上/下拉（仅 gpio 线，按键场景用 pullup）

    // NVIC 优先级（0~15，0 最高）。字段命名对齐 UsartPortConfig 的
    // preempt_priority + sub_priority 拆解方式（原 GD32 版为单 priority 字段）。
    uint32_t preempt_priority = 5;       ///< 抢占优先级（默认 5）
    uint32_t sub_priority     = 0;       ///< 子优先级（默认 0）
};

/**
 * @brief 外部中断（EXTI）—— F103 全 19 线（0~18）
 *
 *   // GPIO 线：按键下降沿 + 内部上拉
 *   static exti_port btn({.port = GPIOA, .pin = pin0,
 *                         .trigger = exti_trigger::falling,
 *                         .pull = pullup});
 *   if (btn.init()) {                 // false = 线被占用或参数非法
 *       btn.on_interrupt([](void*) { led.toggle(); });
 *       btn.enable();
 *   }
 *
 *   // 系统线：PVD 欠压检测（阈值使能见 exti_line_id::pvd 注释）
 *   static exti_port pvd({.line_type = exti_line_id::pvd});
 *   if (pvd.init())
 *       pvd.on_interrupt(掉电处理回调);
 *
 * 资源规则：EXTI 线全局唯一（PA0 与 PB0 同为线 0），
 * 后注册者 init() 返回 false，防止同线互相覆盖。
 */
class exti_port {
public:
    using callback_t = void (*)(void *user_data);

    explicit exti_port(const ExtiConfig &cfg);
    ~exti_port();

    exti_port(const exti_port &) = delete;
    exti_port &operator=(const exti_port &) = delete;

    /**
     * @brief 初始化并注册 EXTI 线（幂等）
     * @return true = 成功；false = 线路被占用或参数非法（见类注释）
     */
    bool init();
    void deinit();
    [[nodiscard]] bool is_initialized() const noexcept { return _initialized; }

    /** @brief 注册中断回调（关中断原子写，防止 ISR 读到撕裂的 cb+data） */
    void on_interrupt(callback_t cb, void *data = nullptr);

    /**
     * @brief 使能 / 失能本线中断（EXTI IMR 位门控，组 NVIC 保持开启）
     * @note  F1 组中断（EXTI9_5/EXTI15_10）为多线共享，此处按线操作，
     *        避免原版按组开关 NVIC 时误伤同组其它实例
     */
    void enable();
    void disable();

    /** @brief 软件触发中断（调试用；线被 disable() 掩蔽时先置 pending，enable() 后触发） */
    void software_trigger();

    /** @brief 当前 EXTI 线号 0~18（调试用） */
    [[nodiscard]] uint8_t line() const noexcept { return _line; }

    // ISR 访问（friend）
    friend void _exti_fire(exti_port *p);

private:
    uint8_t   _resolve_line() const;
    IRQn_Type _get_irq() const;
    void      _register_isr();
    void      _unregister_isr();

    io_ctrl     _pin;
    ExtiConfig  _cfg;
    callback_t  _cb;
    void       *_cb_data;
    bool        _initialized;
    uint8_t     _line;             // EXTI 线号 0~18
    EXTI_HandleTypeDef _hexti;     // HAL EXTI 句柄（承载线编码 EXTI_LINE_n）
};

#endif
