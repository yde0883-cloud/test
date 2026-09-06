#include "delay.h"

static uint32_t fac_us = 0;

void delay_init(void)
{
    SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);
    fac_us = SystemCoreClock / 8000000;  // 72MHz / 8 = 9
}

void delay_us(uint32_t us)
{
    uint32_t ticks = us * fac_us;
    SysTick->LOAD = ticks - 1;
    SysTick->VAL = 0;
    SysTick->CTRL = SysTick_CTRL_ENABLE_Msk;
    while ((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) == 0);
    SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;
}

void delay_ms(uint32_t ms)
{
    while (ms--) delay_us(1000);
}