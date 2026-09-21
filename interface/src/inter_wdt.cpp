// ============================================================
// @platform STM32F1xx（当前平台）
//   基于 STM32 HAL（HAL_IWDG / HAL_WWDG）实现，公共接口签名不变。
//   原 GD32 标准库映射：
//     - fwdgt_write_enable/config/enable → HAL_IWDG_Init
//     - fwdgt_counter_reload             → HAL_IWDG_Refresh
//     - rcu_periph_clock_enable(RCU_WWDGT) → __HAL_RCC_WWDG_CLK_ENABLE()
//     - wwdgt_config/counter_update      → HAL_WWDG_Init / HAL_WWDG_Refresh
// ============================================================

#include "inter_wdt.hpp"

// 句柄常驻（IWDG / WWDG 各一个，符合硬件的单例特性）
namespace {
IWDG_HandleTypeDef g_hiwdg;
WWDG_HandleTypeDef g_hwwdg;
} // namespace

// ════════════════════════════════════════════════════════════
//  IWDG
// ════════════════════════════════════════════════════════════

void wdt_port::iwdg_init(uint32_t prescaler, uint16_t reload)
{
    g_hiwdg.Instance = IWDG;
    g_hiwdg.Init.Prescaler = prescaler;
    g_hiwdg.Init.Reload = reload;

    // IWDG 时钟来自 LSI：HAL_IWDG_Init 会自动使能 LSI 并等待 PVU/RVU 更新完成
    (void)HAL_IWDG_Init(&g_hiwdg);
}

void wdt_port::iwdg_feed()
{
    (void)HAL_IWDG_Refresh(&g_hiwdg);
}

// ════════════════════════════════════════════════════════════
//  WWDG
// ════════════════════════════════════════════════════════════

void wdt_port::wwdg_init(uint32_t prescaler, uint8_t window, uint8_t counter)
{
    // WWDG 计数器时钟派生自 APB1(PCLK1)，需先开外设时钟
    __HAL_RCC_WWDG_CLK_ENABLE();

    g_hwwdg.Instance = WWDG;
    g_hwwdg.Init.Prescaler = prescaler;
    g_hwwdg.Init.Window = window;
    g_hwwdg.Init.Counter = counter;
    g_hwwdg.Init.EWIMode = WWDG_EWI_DISABLE;

    (void)HAL_WWDG_Init(&g_hwwdg);
}

void wdt_port::wwdg_feed(uint8_t counter)
{
    // HAL_WWDG_Refresh 写入的是句柄里的 Counter，这里同步为本次喂狗值
    g_hwwdg.Init.Counter = counter;
    (void)HAL_WWDG_Refresh(&g_hwwdg);
}
