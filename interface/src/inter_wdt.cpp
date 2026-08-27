// ============================================================
// @platform GD32F4xx（当前平台）
// ============================================================

#include "inter_wdt.hpp"

#include "gd32f4xx.h"
#include "gd32f4xx_rcu.h"
#include "gd32f4xx_fwdgt.h"
#include "gd32f4xx_wwdgt.h"

// ════════════════════════════════════════════════════════════
//  IWDG
// ════════════════════════════════════════════════════════════

void wdt_port::iwdg_init(uint32_t prescaler, uint16_t reload)
{
    // LSI 已经由系统启动使能（或在 RCC 中默认使能）
    fwdgt_write_enable();
    fwdgt_config(reload, prescaler);
    fwdgt_enable();
}

void wdt_port::iwdg_feed()
{
    fwdgt_counter_reload();
}

// ════════════════════════════════════════════════════════════
//  WWDG
// ════════════════════════════════════════════════════════════

void wdt_port::wwdg_init(uint32_t prescaler, uint8_t window, uint8_t counter)
{
    rcu_periph_clock_enable(RCU_WWDGT);
    wwdgt_config(counter, window, prescaler);
    wwdgt_enable();
}

void wdt_port::wwdg_feed(uint8_t counter)
{
    wwdgt_counter_update(counter);
}
