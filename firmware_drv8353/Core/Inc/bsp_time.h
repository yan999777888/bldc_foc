/**
  ******************************************************************************
  * @file    bsp_time.h
  * @brief   TIM2 作 1MHz 自由计数器，提供微秒级时基（32 位，71 分钟回绕）
  ******************************************************************************
  */

#ifndef __BSP_TIME_H
#define __BSP_TIME_H

#include "bsp_pinmap.h"

void     TIME_Init(void);
uint32_t TIME_Micros(void);

/* 两个时间戳之差（自动处理 32 位回绕） */
static inline int32_t TIME_Diff(uint32_t now, uint32_t then)
{
    return (int32_t)(now - then);
}

void     TIME_DelayUs(uint32_t us);

#endif /* __BSP_TIME_H */
