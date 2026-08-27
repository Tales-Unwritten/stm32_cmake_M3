#include "delay.h"

// 基于系统时钟72MHz

void delay_Ccount_2x(unsigned int count)
{
    while (count--)
        ;
}
void delay_us(unsigned int us)
{
    us = 9 * us;
    while (us--)
        ;
}
void delay_ms(unsigned int ms)
{
    ms = 8000 * ms;
    while (ms--)
        ;
}
