/**
 * @file    boot_main.cpp
 * @brief   IAP Bootloader 入口
 *
 * 中断向量补充：SysTick/HardFault handler 原在 app 文件 gd32f4xx_it.c，
 * boot 不编入该文件，在此自行定义（外设 ISR 由 inter_usart 等提供）。
 */

#include "iap_boot.hpp"
#include "systick.h"

extern "C" int main(void)
{
    Iap_Boot_Run();   /* 不返回：跳转 app 或死等升级 */
    return 0;
}

/* ── SysTick 1kHz 时基（systick.c 的 tick_increment 递增 get_tick） ── */
extern "C" void SysTick_Handler(void)
{
    tick_increment();
}

/* ── HardFault：死循环，IWDG（跳转前使能）兜底复位 ── */
extern "C" void HardFault_Handler(void)
{
    for (;;)
    {
    }
}
