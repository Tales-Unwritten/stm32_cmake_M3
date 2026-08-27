#pragma once
// ============================================================
// @platform GD32F4xx
// ============================================================

#ifdef __cplusplus

#include <cstdint>
#include "gd32f4xx.h"

struct RtcTime {
    uint16_t year;    ///< 2000~2099
    uint8_t  month;   ///< 1~12
    uint8_t  day;     ///< 1~31
    uint8_t  hour;    ///< 0~23
    uint8_t  minute;  ///< 0~59
    uint8_t  second;  ///< 0~59
};

/**
 * @brief RTC 日历 + 闹钟
 *
 *   rtc_port::init();
 *   rtc_port::set_time({2026, 5, 30, 23, 59, 50});
 *   RtcTime now; rtc_port::get_time(now);
 */
class rtc_port {
public:
    using alarm_cb_t = void (*)(void *);

    static void init();
    static void deinit();

    static void set_time(const RtcTime &t);
    static void get_time(RtcTime &t);

    // ── 闹钟 ──────────────────────────────────────────
    static void set_alarm(const RtcTime &t);
    static void alarm_enable();
    static void alarm_disable();
    static void on_alarm(alarm_cb_t cb, void *data = nullptr);

    // ISR 可访问
    static alarm_cb_t _alarm_cb;
    static void      *_alarm_data;
};

#endif
