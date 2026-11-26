/**
  ******************************************************************************
  * @file    bsp_drv8353.c
  * @brief   POWER_DRV8353HR_V1 功率板驱动
  *
  * 硬件对应关系：
  *   TIM1_CH1/CH1N -> INHA/INLA -> A 相上下桥
  *   TIM1_CH2/CH2N -> INHB/INLB -> B 相上下桥
  *   TIM1_CH3/CH3N -> INHC/INLC -> C 相上下桥
  *
  * 六步换相为什么要三个相位状态：
  *   任一时刻只有两相导通（一相 PWM、一相常通低边），第三相必须悬空。
  *   "常通低边" = CHx 占空比 0（上桥关、下桥开）；
  *   "悬空"     = 把该通道 CCxE/CCxNE 都清 0，两管都关，靠 OSSR 保证输出为低。
  ******************************************************************************
  */

#include "main.h"
#include "bsp_drv8353.h"

static TIM_HandleTypeDef htim1;

static uint8_t  s_step   = 0;
static uint8_t  s_gate_on = 0;      /* 栅极是否已使能 */
static float    s_duty   = 0.0f;
static uint16_t s_ccr    = 0;

/* 六步换相表：pwm=走 PWM 的相，low=低边常通的相，flo=悬空的相
 * 状态序：100 -> 110 -> 010 -> 011 -> 001 -> 101 -> 100（正转） */
static const struct { uint8_t pwm, low, flo; } STEP_TBL[6] = {
    { PH_C, PH_B, PH_A },   /* step0: C+ B-  霍尔 100 */
    { PH_C, PH_A, PH_B },   /* step1: C+ A-  霍尔 110 */
    { PH_B, PH_A, PH_C },   /* step2: B+ A-  霍尔 010 */
    { PH_B, PH_C, PH_A },   /* step3: B+ C-  霍尔 011 */
    { PH_A, PH_C, PH_B },   /* step4: A+ C-  霍尔 001 */
    { PH_A, PH_B, PH_C },   /* step5: A+ B-  霍尔 101 */
};

/* 霍尔 3bit -> step，无效状态返回 -1
 * 位定义：(HA<<2)|(HB<<1)|HC */
static const int8_t HALL2STEP[8] = {
    -1,  /* 000 无效 */
     4,  /* 001 -> A+ C- */
     2,  /* 010 -> B+ A- */
     3,  /* 011 -> B+ C- */
     0,  /* 100 -> C+ B- */
     5,  /* 101 -> A+ B- */
     1,  /* 110 -> C+ A- */
    -1,  /* 111 无效 */
};

/* ---------------- 寄存器级小工具 ---------------- */

static inline void phase_gate(uint8_t ph, uint8_t on)
{
    uint32_t mE  = (uint32_t)TIM_CCER_CC1E  << (4u * ph);
    uint32_t mNE = (uint32_t)TIM_CCER_CC1NE << (4u * ph);
    if (on) {
        TIM1->CCER |= (mE | mNE);
    } else {
        TIM1->CCER &= ~(mE | mNE);
    }
}

static inline void phase_ccr(uint8_t ph, uint16_t ccr)
{
    switch (ph) {
        case PH_A: TIM1->CCR1 = ccr; break;
        case PH_B: TIM1->CCR2 = ccr; break;
        default:   TIM1->CCR3 = ccr; break;
    }
}

static uint16_t duty_to_ccr(float duty)
{
    if (duty < 0.0f) duty = 0.0f;
    if (duty > DRV_DUTY_MAX) duty = DRV_DUTY_MAX;
    uint32_t arr = TIM1->ARR + 1u;
    uint32_t ccr = (uint32_t)(duty * (float)arr + 0.5f);
    if (ccr > TIM1->ARR) ccr = TIM1->ARR;
    return (uint16_t)ccr;
}

/* 死区：DTG[7:0] x tDTS，tDTS = 1/168MHz = 5.95ns（CKD=0） */
static uint32_t dead_time_from_ns(uint32_t ns)
{
    uint32_t tclk = HAL_RCC_GetPCLK2Freq() * 2u;      /* APB2 定时器时钟 168MHz */
    uint64_t ticks = (uint64_t)ns * (uint64_t)tclk / 1000000000ull;
    if (ticks > 127ull) ticks = 127ull;
    return (uint32_t)ticks;
}

/* ---------------- 初始化 ---------------- */

void DRV_Init(void)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOG_CLK_ENABLE();

    /* DRV_EN：先输出低，保证上电不使能 */
    HAL_GPIO_WritePin(DRV_EN_PORT, DRV_EN_PIN, GPIO_PIN_RESET);
    gpio.Pin   = DRV_EN_PIN;
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(DRV_EN_PORT, &gpio);

    /* FAULT：板载已 4.7k 上拉到 3.3V，这里再加内部上拉做冗余 */
    gpio.Pin  = DRV_FAULT_PIN;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(DRV_FAULT_PORT, &gpio);

    /* ---- TIM1：20kHz，三相互补，带死区 ---- */
    htim1.Instance               = TIM1;
    htim1.Init.Prescaler         = 0;
    htim1.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim1.Init.Period            = (HAL_RCC_GetPCLK2Freq() * 2u) / DRV_PWM_FREQ_HZ - 1u;
    htim1.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.RepetitionCounter = 0;
    htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_PWM_Init(&htim1) != HAL_OK) {
        Error_Handler();
    }

    TIM_BreakDeadTimeConfigTypeDef bdt = {0};
    bdt.OffStateRunMode  = TIM_OSSR_ENABLE;     /* 通道关闭时输出强制到空闲电平 */
    bdt.OffStateIDLEMode = TIM_OSSI_ENABLE;
    bdt.LockLevel        = TIM_LOCKLEVEL_OFF;
    bdt.DeadTime         = dead_time_from_ns(DRV_DEAD_TIME_NS);
    bdt.BreakState       = TIM_BREAK_DISABLE;
    bdt.BreakPolarity    = TIM_BREAKPOLARITY_HIGH;
    bdt.AutomaticOutput  = TIM_AUTOMATICOUTPUT_DISABLE;
    if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &bdt) != HAL_OK) {
        Error_Handler();
    }

    TIM_OC_InitTypeDef oc = {0};
    oc.OCMode       = TIM_OCMODE_PWM1;
    oc.Pulse        = 0;
    oc.OCPolarity   = TIM_OCPOLARITY_HIGH;
    oc.OCNPolarity  = TIM_OCNPOLARITY_HIGH;
    oc.OCFastMode   = TIM_OCFAST_DISABLE;
    oc.OCIdleState  = TIM_OCIDLESTATE_RESET;
    oc.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    HAL_TIM_PWM_ConfigChannel(&htim1, &oc, TIM_CHANNEL_1);
    HAL_TIM_PWM_ConfigChannel(&htim1, &oc, TIM_CHANNEL_2);
    HAL_TIM_PWM_ConfigChannel(&htim1, &oc, TIM_CHANNEL_3);

    s_duty = 0.0f;
    s_ccr  = 0;
    s_step = 0;
    DRV_TimerStart();
    DRV_Coast();
}

/* TIM1 跑起来、MOE 打开，但三相栅极全部关闭（安全态） */
void DRV_TimerStart(void)
{
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    TIM1->CCER = 0;             /* 立刻把三相互补输出全关 */
    s_gate_on = 0;
}

/* ---------------- 使能 / 故障 ---------------- */

void DRV_Enable(void)
{
    HAL_GPIO_WritePin(DRV_EN_PORT, DRV_EN_PIN, GPIO_PIN_SET);
    s_gate_on = 1;
}

void DRV_Disable(void)
{
    DRV_Coast();                                    /* 先关管，再断使能 */
    HAL_GPIO_WritePin(DRV_EN_PORT, DRV_EN_PIN, GPIO_PIN_RESET);
    s_gate_on = 0;
}

uint8_t DRV_IsEnabled(void)
{
    return s_gate_on;
}

uint8_t DRV_IsFault(void)
{
    return (HAL_GPIO_ReadPin(DRV_FAULT_PORT, DRV_FAULT_PIN) == GPIO_PIN_RESET) ? 1u : 0u;
}

/* ---------------- 占空比 ---------------- */

void DRV_SetDuty(float duty)
{
    if (duty < 0.0f) duty = 0.0f;
    if (duty > DRV_DUTY_MAX) duty = DRV_DUTY_MAX;
    s_duty = duty;
    s_ccr  = duty_to_ccr(duty);
    /* 只更新当前走 PWM 的那一相，其余两相 CCR 保持 0 */
    phase_ccr(STEP_TBL[s_step].pwm, s_ccr);
}

float DRV_GetDuty(void)
{
    return s_duty;
}

/* ---------------- 换相 ---------------- */

static void apply_step(uint8_t step)
{
    if (step > 5u) { DRV_Coast(); return; }

    const uint8_t pwm = STEP_TBL[step].pwm;
    const uint8_t low = STEP_TBL[step].low;
    const uint8_t flo = STEP_TBL[step].flo;

    s_step = step;
    s_ccr  = duty_to_ccr(s_duty);

    /* 先写占空比，再开关栅极，避免换相瞬间出毛刺 */
    phase_ccr(pwm, s_ccr);
    phase_ccr(low, 0);
    phase_ccr(flo, 0);

    if (s_gate_on) {
        phase_gate(pwm, 1u);
        phase_gate(low, 1u);
        phase_gate(flo, 0u);
    }
}

void DRV_ApplyStep(uint8_t step)
{
    apply_step(step);
}

void DRV_ApplyHall(uint8_t hall_raw)
{
    int8_t st = HALL2STEP[hall_raw & 0x07u];
    if (st < 0) {
        DRV_Coast();        /* 霍尔状态非法（000/111）：立即关管，别赌 */
        return;
    }
    apply_step((uint8_t)st);
}

/* ---------------- 停止 ---------------- */

void DRV_Coast(void)
{
    TIM1->CCER = 0;         /* 三相互补输出全关，OSSR 保证输出低 */
}

void DRV_Brake(void)
{
    s_ccr = 0;
    phase_ccr(PH_A, 0);
    phase_ccr(PH_B, 0);
    phase_ccr(PH_C, 0);
    if (s_gate_on) {
        phase_gate(PH_A, 1u);
        phase_gate(PH_B, 1u);
        phase_gate(PH_C, 1u);
    }
}

/* ---------------- HAL 回调 ---------------- */

void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim)
{
    if (htim->Instance != TIM1) {
        return;
    }

    GPIO_InitTypeDef gpio = {0};
    __HAL_RCC_TIM1_CLK_ENABLE();

    gpio.Mode      = GPIO_MODE_AF_PP;
    gpio.Pull      = GPIO_NOPULL;
    gpio.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = GPIO_AF1_TIM1;

    /* CH1/CH2/CH3 = PA8/PA9/PA10 */
    gpio.Pin = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10;
    HAL_GPIO_Init(GPIOA, &gpio);

    /* CH1N/CH2N/CH3N = PB13/PB14/PB15 */
    gpio.Pin = GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
    HAL_GPIO_Init(GPIOB, &gpio);
}
