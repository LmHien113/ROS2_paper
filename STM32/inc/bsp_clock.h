#ifndef BSP_CLOCK_H
#define BSP_CLOCK_H

#include "app_config.h"

void     SysTick_Init(void);
uint32_t Millis(void);
void     Delay_UsInit(void);
void     Delay_Us(uint32_t us);
void     Delay_Ms(uint32_t ms);

#endif
