/**
  ******************************************************************************
  * @file    bsp_hall.c
  * @brief   霍尔读取与测速
  *
  * 状态序（正转）：100 -> 110 -> 010 -> 011 -> 001 -> 101 -> 100
  * 相邻两次跳变只变一位，据此判断方向；跳变次数对不上就是毛刺，只更新状态不计数。
  ******************************************************************************
  */

#include "main.h"
#include "bsp_hall.h"
#include "bsp_time.h"

static volatile hall_info_t s_hall;
static volatile uint32_t    s_last_change_ms;
static uint8_t              s_inited;

/* 霍尔 3bit -> 换相步（与 bsp_drv8353.c 的 HALL2STEP 必须一致） */
static const int8_t HALL2STEP[8] = { -1, 4, 2, 3, 0, 5, 1, -1 };

/* ---------------- 弱回调 ---------------- */

__weak void Hall_OnEdge(uint8_t raw)
{
    (void)raw;
}

/* ---------------- 初始化 ---------------- */

void Hall_Init(void)
{
    GPIO_InitTypeDef gpio = {0};

    HALL_GPIO_CLK_ENABLE();
    __HAL_RCC_SYSCFG_CLK_ENABLE();

    gpio.Mode = GPIO_MODE_IT_RISING_FALLING;
    gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;

    gpio.Pin = HALL_A_PIN;
    HAL_GPIO_Init(HALL_A_PORT, &gpio);
    gpio.Pin = HALL_B_PIN;
    HAL_GPIO_Init(HALL_B_PORT, &gpio);
    gpio.Pin = HALL_C_PIN;
    HAL_GPIO_Init(HALL_C_PORT, &gpio);

    s_hall.raw    = Hall_ReadRaw();
    s_hall.step   = HALL2STEP[s_hall.raw & 0x07u];
    s_hall.dir    = 0;
    s_hall.count  = 0;
    s_hall.edges  = 0;
    s_hall.period_us = 0;
    s_hall.last_us   = TIME_Micros();
    s_hall.valid     = 0;
    s_last_change_ms = HAL_GetTick();
    s_inited = 1;

    HAL_NVIC_SetPriority(EXTI9_5_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
}

uint8_t Hall_ReadRaw(void)
{
    uint8_t a = (HAL_GPIO_ReadPin(HALL_A_PORT, HALL_A_PIN) == GPIO_PIN_SET) ? 1u : 0u;
    uint8_t b = (HAL_GPIO_ReadPin(HALL_B_PORT, HALL_B_PIN) == GPIO_PIN_SET) ? 1u : 0u;
    uint8_t c = (HAL_GPIO_ReadPin(HALL_C_PORT, HALL_C_PIN) == GPIO_PIN_SET) ? 1u : 0u;
    return (uint8_t)((a << 2) | (b << 1) | c);
}

/* ---------------- 边沿处理 ---------------- */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin != HALL_A_PIN && GPIO_Pin != HALL_B_PIN && GPIO_Pin != HALL_C_PIN) {
        return;
    }
    if (!s_inited) {
        return;
    }

    uint32_t now = TIME_Micros();
    uint8_t  raw = Hall_ReadRaw();
    int8_t   st  = HALL2STEP[raw & 0x07u];

    if (st < 0) {
        /* 000 / 111：线序错误或某根霍尔断线，不动计数器 */
        s_hall.raw  = raw;
        s_hall.step = -1;
        return;
    }

    uint32_t dt = now - s_hall.last_us;
    if (dt == 0u) {
        dt = 1u;
    }

    if (s_hall.valid && s_hall.step >= 0) {
        int8_t diff = (int8_t)((st - s_hall.step + 6) % 6);
        if (diff == 1) {
            s_hall.dir = 1;
            s_hall.count++;
        } else if (diff == 5) {
            s_hall.dir = -1;
            s_hall.count--;
        } else {
            /* 跳两格以上：毛刺，只更新状态，不计数、不更新时间基准 */
            s_hall.raw  = raw;
            s_hall.step = st;
            return;
        }
    }

    /* 一阶平滑，抑制单周期抖动 */
    if (s_hall.period_us == 0u) {
        s_hall.period_us = dt;
    } else {
        s_hall.period_us = (s_hall.period_us * 3u + dt) / 4u;
    }

    s_hall.last_us = now;
    s_hall.raw     = raw;
    s_hall.step    = st;
    s_hall.edges++;
    s_hall.valid   = 1;
    s_last_change_ms = HAL_GetTick();

    Hall_OnEdge(raw);
}

/* ---------------- 查询 ---------------- */

void Hall_GetCopy(hall_info_t *out)
{
    __disable_irq();
    *out = *(hall_info_t *)&s_hall;
    __enable_irq();
}

float Hall_GetElecFreqHz(void)
{
    uint32_t p = s_hall.period_us;
    if (p == 0u) return 0.0f;
    return 1000000.0f / (6.0f * (float)p);
}

float Hall_GetSpeedRpm(uint8_t pole_pairs)
{
    uint32_t p = s_hall.period_us;
    if (p == 0u || pole_pairs == 0u) return 0.0f;
    return 60000000.0f / ((float)p * 6.0f * (float)pole_pairs);
}

float Hall_GetElecAngleDeg(void)
{
    int8_t st = s_hall.step;
    if (st < 0) return 0.0f;
    return (float)st * 60.0f;
}

void Hall_Reset(void)
{
    __disable_irq();
    s_hall.count    = 0;
    s_hall.edges    = 0;
    s_hall.period_us = 0;
    s_hall.dir      = 0;
    s_hall.valid    = 0;
    s_hall.last_us  = TIME_Micros();
    __enable_irq();
}

uint8_t Hall_IsStalled(uint32_t timeout_ms)
{
    if (!s_hall.valid) {
        return 1u;
    }
    return ((HAL_GetTick() - s_last_change_ms) > timeout_ms) ? 1u : 0u;
}
