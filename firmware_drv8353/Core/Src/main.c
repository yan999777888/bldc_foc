/**
  ******************************************************************************
  * @file    main.c
  * @brief   探索者 STM32F407 + POWER_DRV8353HR_V1 初步驱动
  *
  * 功能：
  *   1) 上电自检：FAULT / 母线电压 / NTC / 三相电流零点
  *   2) KEY0 -> 开环六步（不依赖霍尔，先验证功率级接线）
  *   3) KEY1 -> 霍尔闭环六步（验证霍尔线序与换相表）
  *   4) KEY2 -> 停机
  *   5) 1Hz 打印母线电压/电流/温度/三相电流/霍尔状态/转速
  *   6) 软件保护：过流、过压、欠压、过温、FAULT
  *
  * 注意：本文件不调用 CubeMX 生成的 MX_GPIO_Init，外设全部由 bsp_*.c 初始化。
  ******************************************************************************
  */

#include "main.h"
#include "bsp_pinmap.h"
#include "bsp_time.h"
#include "bsp_hall.h"
#include "bsp_drv8353.h"
#include "bsp_adc.h"
#include "bsp_uart.h"

#include <stdio.h>
#include <math.h>

/* ==========================================================================
 *                              可调参数
 * ========================================================================== */

/* ★电机极对数：断电手转一圈，数霍尔完整循环几次（100->110->...->101->100） */
#define MOTOR_POLE_PAIRS        7u

#define VBUS_MIN_V              18.0f   /* 24V 系统，欠压保护 */
#define VBUS_MAX_V              30.0f   /* 过压保护（板子能到 60V，按电池组设） */
#define IBUS_MAX_A              22.0f   /* 板子标称 26A，留余量 */
#define TEMP_MAX_C              85.0f

#define DUTY_TARGET_OL          0.20f   /* 开环演示占空比 */
#define DUTY_TARGET_HALL        0.25f   /* 霍尔闭环演示占空比 */
#define DUTY_RAMP_PER_MS        0.001f  /* 软起动：每毫秒加 0.001 */

#define OL_ELEC_HZ_START        2.0f    /* 开环起始电频率 */
#define OL_ELEC_HZ_END          15.0f   /* 开环最高电频率 */
#define OL_ELEC_HZ_STEP         0.5f    /* 每 500ms 加多少 */

/* ==========================================================================
 *                              内部变量
 * ========================================================================== */

typedef enum {
    MODE_STOP = 0,
    MODE_OPENLOOP,
    MODE_HALL
} mode_e;

static volatile mode_e g_mode = MODE_STOP;
static float    s_duty        = 0.0f;
static float    s_duty_target = 0.0f;
static float    s_elec_hz     = OL_ELEC_HZ_START;
static uint8_t  s_ol_step     = 0;
static uint32_t s_ol_next_us  = 0;
static uint32_t s_ramp_tick   = 0;
static drv_sample_t s_smp;

/* 按键 */
typedef struct {
    GPIO_TypeDef *port;
    uint16_t      pin;
    uint8_t       last_raw;
    uint8_t       stable;
    uint8_t       cnt;
} btn_t;

static btn_t s_btn[3] = {
    { KEY0_PORT, KEY0_PIN, 1, 1, 0 },
    { KEY1_PORT, KEY1_PIN, 1, 1, 0 },
    { KEY2_PORT, KEY2_PIN, 1, 1, 0 },
};

static void SystemClock_Config(void);
static void bsp_led_key_init(void);
static void btn_scan(void);
static void motor_start(mode_e m);
static void motor_stop(const char *why);
static void task_1ms(void);
static void task_report(void);

/* ==========================================================================
 *                                  主函数
 * ========================================================================== */

int main(void)
{
    HAL_Init();
    SystemClock_Config();

    UART_Init();
    TIME_Init();
    bsp_led_key_init();

    printf("\r\n==============================================\r\n");
    printf(" 探索者F407 + POWER_DRV8353HR_V1 六步换相驱动\r\n");
    printf(" 编译时间: %s %s\r\n", __DATE__, __TIME__);
    printf("==============================================\r\n");

    /* 先关管再使能外设，顺序不能反 */
    DRV_Init();          /* EN=低，TIM1 跑起来但三相栅极全关 */
    Hall_Init();
    ADC_Init();

    HAL_Delay(100);      /* 等 ADC 缓冲稳定 */

    ADC_GetSample(&s_smp);
    printf("[自检] VBUS=%.2fV IBUS=%.2fA NTC=%.1fC\r\n",
           s_smp.vbus, s_smp.ibus, s_smp.temp_c);
    printf("[自检] 三相电流原始码 %u/%u/%u, 霍尔=%u%u%u\r\n",
           s_smp.raw[ADC_IDX_IOUTA], s_smp.raw[ADC_IDX_IOUTB], s_smp.raw[ADC_IDX_IOUTC],
           (Hall_ReadRaw() >> 2) & 1u, (Hall_ReadRaw() >> 1) & 1u, Hall_ReadRaw() & 1u);

    if (DRV_IsFault()) {
        printf("[自检] !! FAULT 为低，驱动芯片报故障，禁止启动 !!\r\n");
        printf("       先查：母线极性、电压范围、电机相线是否短路\r\n");
    } else {
        printf("[自检] FAULT 正常（高电平）\r\n");
    }

    if (s_smp.vbus < VBUS_MIN_V || s_smp.vbus > VBUS_MAX_V) {
        printf("[自检] !! 母线电压 %.2fV 不在 %.0f~%.0fV, 检查电源或分压标定 !!\r\n",
               s_smp.vbus, VBUS_MIN_V, VBUS_MAX_V);
    } else {
        printf("[自检] 母线电压正常\r\n");
    }

    /* 停机状态下标定三相电流零点（理论 1.65V 附近） */
    ADC_CalibrateOffset(64);
    ADC_GetSample(&s_smp);
    printf("[标定] 零点已写入: IA=%.3fA IB=%.3fA IC=%.3fA (应为 0 附近)\r\n",
           s_smp.iout_a, s_smp.iout_b, s_smp.iout_c);

    printf("\r\n操作说明：\r\n");
    printf("  KEY0(PE4) = 开环六步, 电频率 %.0f->%.0fHz, 占空比 %.0f%%\r\n",
           OL_ELEC_HZ_START, OL_ELEC_HZ_END, DUTY_TARGET_OL * 100.0f);
    printf("  KEY1(PE3) = 霍尔闭环六步, 占空比 %.0f%%\r\n", DUTY_TARGET_HALL * 100.0f);
    printf("  KEY2(PE2) = 停机\r\n");
    printf("  ★首次通电请用限流电源, 从 12V/1A 开始\r\n\r\n");

    uint32_t last_1ms = HAL_GetTick();
    uint32_t last_5ms = last_1ms;
    uint32_t last_1s  = last_1ms;

    while (1) {
        uint32_t now = HAL_GetTick();

        if ((uint32_t)(now - last_1ms) >= 1u) {
            last_1ms = now;
            task_1ms();
        }
        if ((uint32_t)(now - last_5ms) >= 5u) {
            last_5ms = now;
            btn_scan();
        }
        if ((uint32_t)(now - last_1s) >= 1000u) {
            last_1s = now;
            task_report();
        }
    }
}

/* ==========================================================================
 *                          霍尔跳变 -> 立即换相
 * ========================================================================== */

void Hall_OnEdge(uint8_t raw)
{
    if (g_mode == MODE_HALL) {
        DRV_ApplyHall(raw);
    }
}

/* ==========================================================================
 *                              1ms 周期任务
 * ========================================================================== */

static void task_1ms(void)
{
    ADC_GetSample(&s_smp);

    /* ---- 软件保护：任何一条触发都立刻停机 ---- */
    if (g_mode != MODE_STOP) {
        if (DRV_IsFault())          { motor_stop("FAULT 拉低");   return; }
        if (s_smp.vbus > VBUS_MAX_V){ motor_stop("母线过压");     return; }
        if (s_smp.vbus < VBUS_MIN_V){ motor_stop("母线欠压");     return; }
        if (s_smp.ibus > IBUS_MAX_A){ motor_stop("母线过流");     return; }
        if (s_smp.temp_c > TEMP_MAX_C){ motor_stop("功率板过温"); return; }
    }

    if (g_mode == MODE_STOP) {
        return;
    }

    /* ---- 占空比软起动 ---- */
    if (s_duty < s_duty_target) {
        s_duty += DUTY_RAMP_PER_MS;
        if (s_duty > s_duty_target) s_duty = s_duty_target;
        DRV_SetDuty(s_duty);
    }

    /* ---- 开环：按电频率自己走步 ---- */
    if (g_mode == MODE_OPENLOOP) {
        uint32_t now = TIME_Micros();
        if ((int32_t)(now - s_ol_next_us) >= 0) {
            s_ol_step = (uint8_t)((s_ol_step + 1u) % 6u);
            DRV_ApplyStep(s_ol_step);

            /* 每 500ms 抬一次电频率，直到上限 */
            if ((uint32_t)(HAL_GetTick() - s_ramp_tick) >= 500u) {
                s_ramp_tick = HAL_GetTick();
                if (s_elec_hz < OL_ELEC_HZ_END) {
                    s_elec_hz += OL_ELEC_HZ_STEP;
                }
            }
            s_ol_next_us = now + (uint32_t)(1000000.0f / (6.0f * s_elec_hz));
        }
    }
}

/* ==========================================================================
 *                             启停控制
 * ========================================================================== */

static void motor_start(mode_e m)
{
    uint8_t raw;

    ADC_GetSample(&s_smp);

    if (DRV_IsFault()) {
        printf("[启动失败] FAULT 为低, 驱动芯片在报故障\r\n");
        return;
    }
    if (s_smp.vbus < VBUS_MIN_V || s_smp.vbus > VBUS_MAX_V) {
        printf("[启动失败] 母线 %.2fV 超出 %.0f~%.0fV\r\n",
               s_smp.vbus, VBUS_MIN_V, VBUS_MAX_V);
        return;
    }

    raw = Hall_ReadRaw();
    if (m == MODE_HALL && (raw == 0u || raw == 7u)) {
        printf("[启动失败] 霍尔状态非法 %u%u%u, 检查霍尔供电/上拉/线序\r\n",
               (raw >> 2) & 1u, (raw >> 1) & 1u, raw & 1u);
        return;
    }

    s_duty        = 0.0f;
    s_duty_target = (m == MODE_HALL) ? DUTY_TARGET_HALL : DUTY_TARGET_OL;
    s_elec_hz     = OL_ELEC_HZ_START;
    s_ramp_tick   = HAL_GetTick();
    Hall_Reset();

    DRV_Enable();
    TIME_DelayUs(20000);            /* 等 DRV8353HR 唤醒并稳定 */

    if (DRV_IsFault()) {
        DRV_Disable();
        printf("[启动失败] 使能后 FAULT 仍为低, 已自动断电\r\n");
        return;
    }

    if (m == MODE_OPENLOOP) {
        s_ol_step    = 0u;
        s_ol_next_us = TIME_Micros();
        DRV_ApplyStep(0u);
        printf("[运行] 开环六步启动, 电频率 %.0f->%.0fHz\r\n",
               OL_ELEC_HZ_START, OL_ELEC_HZ_END);
    } else {
        /* 关键: 静止时没有霍尔跳变, 必须先按当前扇区通电, 电机才动得起来 */
        DRV_ApplyHall(raw);
        printf("[运行] 霍尔闭环启动, 当前霍尔=%u%u%u\r\n",
               (raw >> 2) & 1u, (raw >> 1) & 1u, raw & 1u);
    }

    g_mode = m;
    HAL_GPIO_WritePin(LED_G_PORT, LED_G_PIN, GPIO_PIN_RESET);   /* 绿灯亮 */
    HAL_GPIO_WritePin(LED_R_PORT, LED_R_PIN, GPIO_PIN_SET);
}

static void motor_stop(const char *why)
{
    if (g_mode == MODE_STOP) {
        return;
    }
    g_mode = MODE_STOP;
    s_duty = 0.0f;
    s_duty_target = 0.0f;
    DRV_Disable();                  /* 先关管，再断 EN */

    printf("[停机] 原因: %s\r\n", why);
    HAL_GPIO_WritePin(LED_G_PORT, LED_G_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED_R_PORT, LED_R_PIN, GPIO_PIN_RESET);   /* 红灯亮 */
}

/* ==========================================================================
 *                              按键 / LED
 * ========================================================================== */

static void bsp_led_key_init(void)
{
    GPIO_InitTypeDef gpio = {0};

    KEY_LED_GPIO_CLK_ENABLE();

    /* LED 低电平点亮，初始全灭 */
    HAL_GPIO_WritePin(LED_R_PORT, LED_R_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED_G_PORT, LED_G_PIN, GPIO_PIN_SET);
    gpio.Pin   = LED_R_PIN | LED_G_PIN;
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOF, &gpio);

    gpio.Pin  = KEY0_PIN | KEY1_PIN | KEY2_PIN;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOE, &gpio);
}

static void btn_scan(void)
{
    for (uint8_t i = 0; i < 3u; i++) {
        uint8_t raw = (HAL_GPIO_ReadPin(s_btn[i].port, s_btn[i].pin) == GPIO_PIN_SET) ? 1u : 0u;

        if (raw != s_btn[i].last_raw) {
            s_btn[i].last_raw = raw;
            s_btn[i].cnt = 0;
            continue;
        }
        if (s_btn[i].cnt < 5u) {
            s_btn[i].cnt++;
            continue;
        }
        if (raw != s_btn[i].stable) {
            s_btn[i].stable = raw;
            if (raw == 0u) {                    /* 按下沿 */
                if (i == 0u)      motor_start(MODE_OPENLOOP);
                else if (i == 1u) motor_start(MODE_HALL);
                else              motor_stop("按键停机");
            }
        }
    }
}

/* ==========================================================================
 *                             1Hz 数据打印
 * ========================================================================== */

static void task_report(void)
{
    hall_info_t h;
    Hall_GetCopy(&h);

    printf("VBUS=%5.2fV IBUS=%5.2fA NTC=%5.1fC | IA=%6.2f IB=%6.2f IC=%6.2f | "
           "HALL=%u%u%u dir=%+d rpm=%6.1f duty=%3.0f%%%s%s\r\n",
           s_smp.vbus, s_smp.ibus, s_smp.temp_c,
           s_smp.iout_a, s_smp.iout_b, s_smp.iout_c,
           (h.raw >> 2) & 1u, (h.raw >> 1) & 1u, h.raw & 1u,
           h.dir,
           Hall_GetSpeedRpm(MOTOR_POLE_PAIRS),
           s_duty * 100.0f,
           DRV_IsFault() ? " FAULT" : "",
           (g_mode == MODE_STOP) ? " STOP" : ((g_mode == MODE_HALL) ? " HALL" : " OPEN"));
}

/* ==========================================================================
 *                              系统时钟
 * ========================================================================== */

static void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState       = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState   = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM       = 8;
    RCC_OscInitStruct.PLL.PLLN       = 336;
    RCC_OscInitStruct.PLL.PLLP       = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ       = 4;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                     | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
        Error_Handler();
    }
}

void Error_Handler(void)
{
    __disable_irq();
    while (1) {
    }
}
