/**
  ******************************************************************************
  * @file    bsp_drv8353.h
  * @brief   POWER_DRV8353HR_V1 功率板驱动（TIM1 六路互补 PWM + 六步换相）
  ******************************************************************************
  */

#ifndef __BSP_DRV8353_H
#define __BSP_DRV8353_H

#include "bsp_pinmap.h"

#define DRV_PWM_FREQ_HZ        20000u   /* 板子标称 10k~40k，取 20k */
#define DRV_DEAD_TIME_NS       500u     /* 6xPWM 模式死区由 MCU 自己插 */
#define DRV_DUTY_MAX           0.95f    /* 留 5% 余量，方便观测 */

/* 三相编号 */
#define PH_A                   0u
#define PH_B                   1u
#define PH_C                   2u

void    DRV_Init(void);

/* 使能/失能：DRV_Disable() 会先关所有栅极再拉低 EN */
void    DRV_Enable(void);
void    DRV_Disable(void);
uint8_t DRV_IsEnabled(void);

/* FAULT 低电平有效，返回 1 表示当前有故障 */
uint8_t DRV_IsFault(void);

/* 设置当前步的占空比（0.0~1.0），运行中可随时调用 */
void    DRV_SetDuty(float duty);
float   DRV_GetDuty(void);

/* 六步换相：step = 0~5（开环用），hall = 3bit 霍尔原始值（闭环用） */
void    DRV_ApplyStep(uint8_t step);
void    DRV_ApplyHall(uint8_t hall_raw);

/* 三相全部关断（悬空，惯性滑行） */
void    DRV_Coast(void);
/* 三相下桥全通（能耗制动） */
void    DRV_Brake(void);

/* 一上电就把 PWM 定时器跑起来但栅极全关，之后靠 CCER 开关 */
void    DRV_TimerStart(void);

#endif /* __BSP_DRV8353_H */
