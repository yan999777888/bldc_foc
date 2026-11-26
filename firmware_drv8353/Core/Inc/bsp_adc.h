/**
  ******************************************************************************
  * @file    bsp_adc.h
  * @brief   功率板六路模拟量采集（ADC1 + DMA 循环）
  ******************************************************************************
  */

#ifndef __BSP_ADC_H
#define __BSP_ADC_H

#include "bsp_pinmap.h"

typedef struct {
    uint16_t raw[ADC_CH_NUM];
    float    iout_a;        /* A，已扣零点 */
    float    iout_b;
    float    iout_c;
    float    ibus;          /* A，单向 */
    float    vbus;          /* V */
    float    temp_c;        /* 摄氏度 */
    uint8_t  offset_ok;     /* 零点是否已标定 */
} drv_sample_t;

void  ADC_Init(void);

/* 停机状态下调用：采 n 次求平均，作为三相电流零点（理论 VREF/2 ≈ 1.65V） */
void  ADC_CalibrateOffset(uint16_t n);

/* 取一份最新样本（内部读 DMA 缓冲并换算） */
void  ADC_GetSample(drv_sample_t *out);

#endif /* __BSP_ADC_H */
