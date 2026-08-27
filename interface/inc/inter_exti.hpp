#pragma once
// ============================================================
// @platform GD32F4xx
// ============================================================

#ifdef __cplusplus

#include <cstdint>
#include "inter_io_ctrl.hpp"
#include "gd32f4xx.h"

enum class exti_trigger : uint8_t { rising, falling, both };

/**
 * @brief EXTI 线类型（GPIO 线 0~15 + 系统事件线 16~22）
 *
 * 对齐参考实现（王海涛 exti_bsp.c）的系统线能力：
 *  - GPIO 线：任意端口引脚，需配置 SYSCFG 端口选择
 *  - 系统线：LVD / USB 唤醒 / 以太网唤醒 / RTC 时间戳 / RTC 唤醒
 *  - ⚠️ rtc_alarm(17) 的 ISR 由 inter_rtc（rtc_port）管理，本类不注册该线
 */
enum class exti_line_id : uint8_t {
    gpio          = 0,    ///< GPIO 引脚（配合 port/pin）
    lvd           = 16,   ///< EXTI16 低电压检测（LVD）
    rtc_alarm     = 17,   ///< EXTI17 RTC 闹钟（⚠️ inter_rtc 管理，本类不支持）
    usb_fs        = 18,   ///< EXTI18 USB FS 唤醒
    ethernet      = 19,   ///< EXTI19 以太网唤醒（GD32F450/470/407/427）
    usb_hs        = 20,   ///< EXTI20 USB HS 唤醒
    rtc_timestamp = 21,   ///< EXTI21 RTC 时间戳
    rtc_wakeup    = 22,   ///< EXTI22 RTC 唤醒
};

struct ExtiConfig {
    exti_line_id  line_type = exti_line_id::gpio;
    GPIO_TypeDef* port      = nullptr;   ///< GPIOA~GPIOI（仅 gpio 线）
    pin_enum_t    pin       = pin_none;   ///< GPIO_PIN_0 ~ GPIO_PIN_15（仅 gpio 线）
    exti_trigger  trigger  = exti_trigger::rising;  ///< 触发沿（仅 gpio 线；系统线硬件固定上升沿）
    pull_enum_t   pull     = nopull;      ///< 上/下拉（仅 gpio 线，按键场景用 pullup）
    uint8_t       priority = 5;                     ///< NVIC 优先级 0~15（0 为最高抢占级，默认 5）
};

/**
 * @brief 外部中断（EXTI）—— 全 23 线支持
 *
 *   // GPIO 线：按键下降沿 + 内部上拉
 *   static exti_port btn({.port = GPIOA, .pin = pin0,
 *                         .trigger = exti_trigger::falling,
 *                         .pull = pullup});
 *   if (btn.init()) {                 // false = 线被占用
 *       btn.on_interrupt([](void*) { led.toggle(); });
 *       btn.enable();
 *   }
 *
 *   // 系统线：LVD 低电压检测
 *   static exti_port lvd({.line_type = exti_line_id::lvd});
 *   lvd.init();
 *   lvd.on_interrupt(lvd 掉电处理回调);
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

    /** @brief 使能 / 失能 NVIC 中断 */
    void enable();
    void disable();

    /** @brief 软件触发中断（调试用，等价参考实现的 Soft_Interrupt） */
    void software_trigger();

    /** @brief 当前 EXTI 线号 0~22（调试用） */
    [[nodiscard]] uint8_t line() const noexcept { return _line; }

    // ISR 访问（friend）
    friend void _exti_fire(exti_port *p);

private:
    uint8_t   _resolve_line() const;
    void      _config_syscfg();
    IRQn_Type _get_irq() const;
    void      _register_isr();
    void      _unregister_isr();

    io_ctrl     _pin;
    ExtiConfig  _cfg;
    callback_t  _cb;
    void       *_cb_data;
    bool        _initialized;
    uint8_t     _line;       // EXTI 线号 0~22
};

#endif
