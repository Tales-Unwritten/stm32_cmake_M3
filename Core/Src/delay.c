#include "delay.h"

// 基于系统时钟72MHz

void delay_Ccount_2x(unsigned int count)
{
    volatile unsigned int count_t = count;
    while (count_t--)
        ;
}
void delay_us(unsigned int us)
{
    volatile unsigned int us_t = 9 * us;
    while (us_t--)
        ;
}
void delay_ms(unsigned int ms)
{
    volatile unsigned int ms_t = 8000 * ms;
    while (ms_t--)
        ;
}
