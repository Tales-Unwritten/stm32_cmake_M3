#pragma once
// ============================================================
// @platform STM32F1xx（当前平台）
// ============================================================

#ifdef __cplusplus

#include <cstdint>
#include "stm32f1xx_hal.h"

/**
 * @brief 看门狗（IWDG + WWDG）
 *
 * IWDG 是单例，使用静态方法：
 *   wdt_port::iwdg_init(IWDG_PRESCALER_64, 4095);  // LSI≈40kHz → ~6.5s 超时
 *   wdt_port::iwdg_feed();  // 主循环中喂狗
 *
 * WWDG（窗口看门狗，计数器时钟 = PCLK1/4096/分频）：
 *   wdt_port::wwdg_init(WWDG_PRESCALER_8, 0x5F, 0x7F);
 *   wdt_port::wwdg_feed(0x7F);  // 必须在窗口允许区间内喂狗
 */
class wdt_port {
public:
    // ── IWDG（独立看门狗，LSI ~40kHz） ──────────────────────
    /// @param prescaler IWDG_PRESCALER_4 ~ IWDG_PRESCALER_256
    /// @param reload    0 ~ 0x0FFF
    static void iwdg_init(uint32_t prescaler, uint16_t reload);
    static void iwdg_feed();

    // ── WWDG（窗口看门狗，PCLK1） ──────────────────────────
    /// @param prescaler WWDG_PRESCALER_1/2/4/8
    /// @param window    0x40 ~ 0x7F
    /// @param counter   0x40 ~ 0x7F
    static void wwdg_init(uint32_t prescaler, uint8_t window, uint8_t counter);
    static void wwdg_feed(uint8_t counter);
};

#endif
