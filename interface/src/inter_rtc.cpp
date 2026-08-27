// ============================================================
// @platform GD32F4xx（当前平台）
// ============================================================

#include "inter_rtc.hpp"

#include "gd32f4xx.h"
#include "gd32f4xx_rcu.h"
#include "gd32f4xx_rtc.h"
#include "gd32f4xx_pmu.h"
#include "gd32f4xx_misc.h"

rtc_port::alarm_cb_t rtc_port::_alarm_cb = nullptr;
void *rtc_port::_alarm_data = nullptr;

void rtc_port::init()
{
    // 使能电源和备份域访问
    rcu_periph_clock_enable(RCU_PMU);
    pmu_backup_write_enable();

    // 选择 RTC 时钟源为 LXTAL（外部 32.768kHz）
    // 注意：需确保硬件上 LXTAL 晶振已连接并正常工作
    rcu_rtc_clock_config(RCU_RTCSRC_LXTAL);
    rcu_periph_clock_enable(RCU_RTC);
    rtc_register_sync_wait();

    // 配置预分频器: 32768Hz → 1Hz
    rtc_parameter_struct rtc_cfg;
    rtc_cfg.factor_asyn = 0x7F;    // 32768/(127+1) = 256Hz
    rtc_cfg.factor_syn  = 0xFF;    // 256/(255+1) = 1Hz
    rtc_cfg.display_format = RTC_24HOUR;
    rtc_cfg.am_pm = RTC_AM;
    rtc_init(&rtc_cfg);
}

void rtc_port::deinit()
{
    rtc_register_sync_wait();
}

void rtc_port::set_time(const RtcTime &t)
{
    rtc_register_sync_wait();

    rtc_parameter_struct rtc_cfg;
    rtc_cfg.year   = t.year;
    rtc_cfg.month  = t.month;
    rtc_cfg.date   = t.day;
    rtc_cfg.hour   = t.hour;
    rtc_cfg.minute = t.minute;
    rtc_cfg.second = t.second;
    rtc_cfg.display_format = RTC_24HOUR;
    rtc_cfg.am_pm = RTC_AM;
    rtc_init(&rtc_cfg);
}

void rtc_port::get_time(RtcTime &t)
{
    rtc_parameter_struct rtc_cfg;
    rtc_current_time_get(&rtc_cfg);
    t.year   = rtc_cfg.year;
    t.month  = rtc_cfg.month;
    t.day    = rtc_cfg.date;
    t.hour   = rtc_cfg.hour;
    t.minute = rtc_cfg.minute;
    t.second = rtc_cfg.second;
}

void rtc_port::set_alarm(const RtcTime &t)
{
    rtc_register_sync_wait();

    rtc_alarm_struct alarm;
    alarm.alarm_mask  = RTC_ALARM_DATE_MASK;
    alarm.weekday_or_date = RTC_ALARM_DATE_SELECTED;
    alarm.alarm_day   = t.day;
    alarm.alarm_hour  = t.hour;
    alarm.alarm_minute= t.minute;
    alarm.alarm_second= t.second;
    alarm.am_pm       = RTC_AM;

    rtc_alarm_config(RTC_ALARM0, &alarm);
}

void rtc_port::alarm_enable()
{
    rtc_interrupt_enable(RTC_INT_ALARM0);
    nvic_irq_enable(RTC_Alarm_IRQn, 0, 0);
}

void rtc_port::alarm_disable()
{
    rtc_interrupt_disable(RTC_INT_ALARM0);
    nvic_irq_disable(RTC_Alarm_IRQn);
}

void rtc_port::on_alarm(alarm_cb_t cb, void *data)
{
    _alarm_cb = cb;
    _alarm_data = data;
}

extern "C" void RTC_Alarm_IRQHandler(void)
{
    if (rtc_flag_get(RTC_FLAG_ALRM0) == SET) {
        rtc_flag_clear(RTC_FLAG_ALRM0);
        if (rtc_port::_alarm_cb) {
            rtc_port::_alarm_cb(rtc_port::_alarm_data);
        }
    }
}
