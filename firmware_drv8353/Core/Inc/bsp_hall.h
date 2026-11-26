/**
  ******************************************************************************
  * @file    bsp_hall.h
  * @brief   霍尔信号读取（EXTI 边沿中断）+ 转速/方向/位置估算
  ******************************************************************************
  */

#ifndef __BSP_HALL_H
#define __BSP_HALL_H

#include "bsp_pinmap.h"

typedef struct {
    uint8_t  raw;        /* 当前 3bit 霍尔值 (HA<<2)|(HB<<1)|HC */
    int8_t   step;       /* 对应换相步 0~5，-1 表示非法态 */
    int8_t   dir;        /* +1 正转，-1 反转，0 未定 */
    int32_t  count;      /* 电角度累计步数，每步 60 度电角度 */
    uint32_t edges;      /* 累计跳变次数 */
    uint32_t period_us;  /* 平滑后的跳变间隔（微秒） */
    uint32_t last_us;    /* 最近一次跳变的时间戳 */
    uint8_t  valid;      /* 至少收到过一次有效跳变 */
} hall_info_t;

void               Hall_Init(void);
void               Hall_GetCopy(hall_info_t *out);
uint8_t            Hall_ReadRaw(void);

/* 电频率 = 1 / (6 x 跳变间隔) */
float              Hall_GetElecFreqHz(void);
/* 机械转速 rpm = 60e6 / (period_us x 6 x 极对数) */
float              Hall_GetSpeedRpm(uint8_t pole_pairs);
/* 电角度（0~360，按步数估算，不含步内插值） */
float              Hall_GetElecAngleDeg(void);

void               Hall_Reset(void);
/* 超过 timeout_ms 没有跳变 -> 认为堵转/停转 */
uint8_t            Hall_IsStalled(uint32_t timeout_ms);

/* 每次霍尔跳变后回调（弱函数，默认空实现，在 main.c 里接管换相） */
void               Hall_OnEdge(uint8_t raw);

#endif /* __BSP_HALL_H */
