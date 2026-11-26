/**
  ******************************************************************************
  * @file    bsp_time.c
  * @brief   TIM2 1MHz 自由运行计数器
  * @note    TIM2 在 APB1 上，APB1=42MHz 时定时器时钟翻倍 = 84MHz，分频 84 得 1MHz
  ******************************************************************************
  */

#include "main.h"
#include "bsp_time.h"

void TIME_Init(void)
{
    __HAL_RCC_TIM2_CLK_ENABLE();

    TIM2->CR1  = 0;
    TIM2->PSC  = (HAL_RCC_GetPCLK1Freq() * 2u) / 1000000u - 1u;
    TIM2->ARR  = 0xFFFFFFFFu;
    TIM2->EGR  = TIM_EGR_UG;            /* 立即把 PSC 装进计数器 */
    TIM2->SR   = 0;
    TIM2->CR1 |= TIM_CR1_CEN;
}

uint32_t TIME_Micros(void)
{
    return TIM2->CNT;
}

void TIME_DelayUs(uint32_t us)
{
    uint32_t start = TIME_Micros();
    while ((uint32_t)(TIME_Micros() - start) < us) {
        __NOP();
    }
}
