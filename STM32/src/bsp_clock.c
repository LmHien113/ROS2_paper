#include "bsp_clock.h"

static volatile uint32_t g_ms = 0U;

/* Xoa SysTick_Handler trong stm32f4xx_it.c neu bao "multiple definition". */
void SysTick_Handler(void)
{
    g_ms++;
}

void SysTick_Init(void)
{
    SystemCoreClockUpdate();
    if (SysTick_Config(SystemCoreClock / 1000UL) != 0UL)
    {
        for (;;) { }
    }
    /* Uu tien cao nhat: log UART khong duoc lam tre dong ho. */
    NVIC_SetPriority(SysTick_IRQn, 0U);
}

uint32_t Millis(void)
{
    return g_ms;
}

/* TIM1 dem 1 MHz. Clock TIM1 = 2 x PCLK2 vi APB2 bi chia > 1. */
void Delay_UsInit(void)
{
    TIM_TimeBaseInitTypeDef tim;
    RCC_ClocksTypeDef clocks;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
    RCC_GetClocksFreq(&clocks);

    TIM_TimeBaseStructInit(&tim);
    tim.TIM_Period = 0xFFFFU;
    tim.TIM_Prescaler = (uint16_t)((clocks.PCLK2_Frequency * 2UL) / 1000000UL - 1UL);
    TIM_TimeBaseInit(TIM1, &tim);
    TIM_SetCounter(TIM1, 0U);
    TIM_Cmd(TIM1, ENABLE);
}

/* Toi da 65535 us. */
void Delay_Us(uint32_t us)
{
    if (us > 0xFFFFUL)
        us = 0xFFFFUL;

    TIM_SetCounter(TIM1, 0U);
    while (TIM_GetCounter(TIM1) < us) { }
}

/* Cat theo khoi 60 ms de chi phi goi ham khong cong don moi ms. */
void Delay_Ms(uint32_t ms)
{
    for (; ms >= 60UL; ms -= 60UL)
        Delay_Us(60000UL);

    Delay_Us(ms * 1000UL);
}
