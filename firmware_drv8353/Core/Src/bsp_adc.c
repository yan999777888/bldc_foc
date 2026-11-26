/**
  ******************************************************************************
  * @file    bsp_adc.c
  * @brief   功率板六路模拟量：三相相电流 / 母线电流 / 母线电压 / NTC
  *
  * 换算系数（来自说明书第 4 章与原理图）：
  *   相电流  2mΩ x 5V/V   -> 10mV/A，零点 = VREF/2 ≈ 1.65V（需实测标定）
  *   母线电流 2mΩ x 20V/V -> 40mV/A，单向
  *   母线电压 124k/4.7k    -> 1/27.38，90V 对应满量程 3.3V
  *   NTC      10k 上拉 + 10k/B3950
  ******************************************************************************
  */

#include "main.h"
#include "bsp_adc.h"
#include <math.h>

ADC_HandleTypeDef  hadc1;
DMA_HandleTypeDef  hdma_adc1;

static uint16_t s_adc_buf[ADC_CH_NUM];

/* 零点（单位 V），三相分别标定 */
static float s_off[3] = {1.65f, 1.65f, 1.65f};
static uint8_t s_offset_ok = 0;

#define ADC_VREF_V      3.3f
#define ADC_LSB_V       (ADC_VREF_V / 4095.0f)
#define R_SHUNT_OHM     0.002f
#define GAIN_PHASE      5.0f
#define GAIN_BUS        20.0f
#define VBUS_DIV_RATIO  ((124.0f + 4.7f) / 4.7f)    /* ≈27.383 */
#define NTC_R25_OHM     10000.0f
#define NTC_B           3950.0f

static inline float raw_to_v(uint16_t raw)
{
    return (float)raw * ADC_LSB_V;
}

/* ---------------- 初始化 ---------------- */

void ADC_Init(void)
{
    ADC_ChannelConfTypeDef ch = {0};

    hadc1.Instance                   = ADC1;
    hadc1.Init.ClockPrescaler        = ADC_CLOCK_SYNC_PCLK_DIV4;   /* 84/4 = 21MHz */
    hadc1.Init.Resolution            = ADC_RESOLUTION_12B;
    hadc1.Init.ScanConvMode          = ENABLE;
    hadc1.Init.ContinuousConvMode    = ENABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.NbrOfDiscConversion   = 0;
    hadc1.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc1.Init.ExternalTrigConv      = ADC_SOFTWARE_START;
    hadc1.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion       = ADC_CH_NUM;
    hadc1.Init.DMAContinuousRequests = ENABLE;
    hadc1.Init.EOCSelection          = ADC_EOC_SEQ_CONV;
    if (HAL_ADC_Init(&hadc1) != HAL_OK) {
        Error_Handler();
    }

    /* rank 顺序必须和 ADC_IDX_xxx 一致 */
    ch.SamplingTime = ADC_SAMPLETIME_480CYCLES;

    ch.Channel = ADC1_CH_IOUTA; ch.Rank = 1; HAL_ADC_ConfigChannel(&hadc1, &ch);
    ch.Channel = ADC1_CH_IOUTB; ch.Rank = 2; HAL_ADC_ConfigChannel(&hadc1, &ch);
    ch.Channel = ADC1_CH_IOUTC; ch.Rank = 3; HAL_ADC_ConfigChannel(&hadc1, &ch);
    ch.Channel = ADC1_CH_CBUS;  ch.Rank = 4; HAL_ADC_ConfigChannel(&hadc1, &ch);
    ch.Channel = ADC1_CH_VBUS;  ch.Rank = 5; HAL_ADC_ConfigChannel(&hadc1, &ch);
    ch.Channel = ADC1_CH_NTC;   ch.Rank = 6; HAL_ADC_ConfigChannel(&hadc1, &ch);

    if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)s_adc_buf, ADC_CH_NUM) != HAL_OK) {
        Error_Handler();
    }
}

/* ---------------- 零点标定 ---------------- */

void ADC_CalibrateOffset(uint16_t n)
{
    if (n == 0u) return;

    float sa = 0.0f, sb = 0.0f, sc = 0.0f;
    for (uint16_t i = 0; i < n; i++) {
        sa += raw_to_v(s_adc_buf[ADC_IDX_IOUTA]);
        sb += raw_to_v(s_adc_buf[ADC_IDX_IOUTB]);
        sc += raw_to_v(s_adc_buf[ADC_IDX_IOUTC]);
        HAL_Delay(2);
    }
    s_off[0] = sa / (float)n;
    s_off[1] = sb / (float)n;
    s_off[2] = sc / (float)n;
    s_offset_ok = 1;
}

/* ---------------- 采样 ---------------- */

void ADC_GetSample(drv_sample_t *out)
{
    /* DMA 在后台循环刷新，这里拷一份快照 */
    for (uint8_t i = 0; i < ADC_CH_NUM; i++) {
        out->raw[i] = s_adc_buf[i];
    }

    float va = raw_to_v(out->raw[ADC_IDX_IOUTA]);
    float vb = raw_to_v(out->raw[ADC_IDX_IOUTB]);
    float vc = raw_to_v(out->raw[ADC_IDX_IOUTC]);

    out->iout_a = (va - s_off[0]) / (R_SHUNT_OHM * GAIN_PHASE);
    out->iout_b = (vb - s_off[1]) / (R_SHUNT_OHM * GAIN_PHASE);
    out->iout_c = (vc - s_off[2]) / (R_SHUNT_OHM * GAIN_PHASE);

    out->ibus = raw_to_v(out->raw[ADC_IDX_CBUS]) / (R_SHUNT_OHM * GAIN_BUS);
    out->vbus = raw_to_v(out->raw[ADC_IDX_VBUS]) * VBUS_DIV_RATIO;

    float vn = raw_to_v(out->raw[ADC_IDX_NTC]);
    if (vn <= 0.01f || vn >= (ADC_VREF_V - 0.01f)) {
        out->temp_c = -99.0f;                      /* 未接 NTC 或开路 */
    } else {
        float r = NTC_R25_OHM * vn / (ADC_VREF_V - vn);
        float inv_t = (1.0f / 298.15f) + (logf(r / NTC_R25_OHM) / NTC_B);
        out->temp_c = (1.0f / inv_t) - 273.15f;
    }

    out->offset_ok = s_offset_ok;
}

/* ---------------- HAL 回调 ---------------- */

void HAL_ADC_MspInit(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance != ADC1) {
        return;
    }

    GPIO_InitTypeDef gpio = {0};
    __HAL_RCC_ADC1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    gpio.Mode  = GPIO_MODE_ANALOG;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;

    gpio.Pin = ADC_IOUTA_PIN | ADC_IOUTB_PIN;
    HAL_GPIO_Init(GPIOA, &gpio);
    gpio.Pin = ADC_IOUTC_PIN | ADC_CBUS_PIN;
    HAL_GPIO_Init(GPIOB, &gpio);
    gpio.Pin = ADC_VBUS_PIN | ADC_NTC_PIN;
    HAL_GPIO_Init(GPIOC, &gpio);

    /* DMA2_Stream0 / Channel0 = ADC1 */
    __HAL_RCC_DMA2_CLK_ENABLE();
    hdma_adc1.Instance                 = DMA2_Stream0;
    hdma_adc1.Init.Channel             = DMA_CHANNEL_0;
    hdma_adc1.Init.Direction           = DMA_PERIPH_TO_MEMORY;
    hdma_adc1.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma_adc1.Init.MemInc              = DMA_MINC_ENABLE;
    hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_adc1.Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD;
    hdma_adc1.Init.Mode                = DMA_CIRCULAR;
    hdma_adc1.Init.Priority            = DMA_PRIORITY_HIGH;
    hdma_adc1.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_adc1) != HAL_OK) {
        Error_Handler();
    }
    __HAL_LINKDMA(hadc, DMA_Handle, hdma_adc1);
}
