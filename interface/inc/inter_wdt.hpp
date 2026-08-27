#pragma once
// ============================================================
// @platform GD32F4xx
// ============================================================

#ifdef __cplusplus

#include <cstdint>
#include "gd32f4xx.h"

/**
 * @brief 看门狗（IWDG + WWDG）
 *
 * IWDG 是单例，使用静态方法：
 *   wdt_port::iwdg_init(IWDG_PRESCALER_64, 4095);  // ~6.5s 超时
 *   wdt_port::iwdg_feed();  // 主循环中喂狗
 */
class wdt_port {
public:
    // ── IWDG（独立看门狗，LSI ~40kHz） ──────────────────────
    static void iwdg_init(uint32_t prescaler, uint16_t reload);
    static void iwdg_feed();

    // ── WWDG（窗口看门狗，PCLK1） ──────────────────────────
    static void wwdg_init(uint32_t prescaler, uint8_t window, uint8_t counter);
    static void wwdg_feed(uint8_t counter);
};

#endif
