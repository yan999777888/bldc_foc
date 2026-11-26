/**
  ******************************************************************************
  * @file    bsp_pinmap.h
  * @brief   探索者 STM32F407 + POWER_DRV8353HR_V1 引脚分配表
  * @note    每条都写了"使用前提"，接线前逐条确认，否则会和板上外设打架
  ******************************************************************************
  */

#ifndef __BSP_PINMAP_H
#define __BSP_PINMAP_H

#include "stm32f4xx_hal.h"

/* ==========================================================================
 * 1. 功率板 P2 接口（3.3V 逻辑，6xPWM 模式）
 * --------------------------------------------------------------------------
 *  INHA / INLA  ->  PA8  / PB13   TIM1_CH1 / CH1N
 *  INHB / INLB  ->  PA9  / PB14   TIM1_CH2 / CH2N
 *  INHC / INLC  ->  PA10 / PB15   TIM1_CH3 / CH3N
 *
 *  前提：
 *   - PA9 / PA10 与板载 CH340(USB转串口) 相连，必须拔掉 P6 跳线帽，
 *     否则 CH340 的 TX 会和 TIM1_CH3 输出对打，可能烧 IO。
 *   - PA8 与 HS0038 红外接收头共用，调试时不要用红外遥控器对着板子。
 *   - PB13/PB14/PB15 分别是 WM8978 SCLK / W25Q128 片选 / LCD 背光，
 *     不用音频、不用 SPI Flash、不插 TFTLCD 模块即可。
 * ========================================================================== */
#define DRV_EN_PORT            GPIOG
#define DRV_EN_PIN             GPIO_PIN_6      /* NRF_CE，不插无线模块即可 */
#define DRV_FAULT_PORT         GPIOG
#define DRV_FAULT_PIN          GPIO_PIN_7      /* NRF_CS，不插无线模块即可 */

/* ==========================================================================
 * 2. 霍尔输入（EXTI 6/7/8，三路必须落在不同 EXTI 线号上）
 * --------------------------------------------------------------------------
 *  前提：不插 OLED/CAMERA 模块（PB6=D5, PB7=VSYNC, PC8=D2），不插 SD 卡。
 *  霍尔输出若是开集，靠 MCU 内部上拉即可；若是推挽也没影响。
 * ========================================================================== */
#define HALL_A_PORT            GPIOB
#define HALL_A_PIN             GPIO_PIN_6
#define HALL_B_PORT            GPIOB
#define HALL_B_PIN             GPIO_PIN_7
#define HALL_C_PORT            GPIOC
#define HALL_C_PIN             GPIO_PIN_8

#define HALL_GPIO_CLK_ENABLE()   do{ __HAL_RCC_GPIOB_CLK_ENABLE(); \
                                     __HAL_RCC_GPIOC_CLK_ENABLE(); }while(0)

/* ==========================================================================
 * 3. 模拟采样（全部用 ADC1，DMA 循环搬运）
 * --------------------------------------------------------------------------
 *  rank  信号        引脚   ADC1 通道   前提
 *   1    IOUTA       PA5    IN5        拔掉 P12 跳线帽（否则接 TPAD）
 *   2    IOUTB       PA4    IN4        不插 CAMERA（DCMI_HREF）
 *   3    IOUTC       PB0    IN8        不插 TFTLCD（触摸 SCK）
 *   4    ADC_CBUS    PB1    IN9        不插 TFTLCD（触摸 PEN）
 *   5    ADC_VBUS    PC3    IN13       不用 WM8978（DACDAT 是 codec 输入）
 *   6    ADC_NTC     PC1    IN11       不用以太网（MDC 是 PHY 输入）
 * ========================================================================== */
#define ADC_IOUTA_PORT         GPIOA
#define ADC_IOUTA_PIN          GPIO_PIN_5
#define ADC_IOUTB_PORT         GPIOA
#define ADC_IOUTB_PIN          GPIO_PIN_4
#define ADC_IOUTC_PORT         GPIOB
#define ADC_IOUTC_PIN          GPIO_PIN_0
#define ADC_CBUS_PORT          GPIOB
#define ADC_CBUS_PIN           GPIO_PIN_1
#define ADC_VBUS_PORT          GPIOC
#define ADC_VBUS_PIN           GPIO_PIN_3
#define ADC_NTC_PORT           GPIOC
#define ADC_NTC_PIN            GPIO_PIN_1

#define ADC1_CH_IOUTA          ADC_CHANNEL_5
#define ADC1_CH_IOUTB          ADC_CHANNEL_4
#define ADC1_CH_IOUTC          ADC_CHANNEL_8
#define ADC1_CH_CBUS           ADC_CHANNEL_9
#define ADC1_CH_VBUS           ADC_CHANNEL_13
#define ADC1_CH_NTC            ADC_CHANNEL_11

/* 采样缓冲下标，与上面 rank 顺序一一对应 */
#define ADC_IDX_IOUTA          0
#define ADC_IDX_IOUTB          1
#define ADC_IDX_IOUTC          2
#define ADC_IDX_CBUS           3
#define ADC_IDX_VBUS           4
#define ADC_IDX_NTC            5
#define ADC_CH_NUM             6

/* ==========================================================================
 * 4. 调试串口 USART3（P10 跳线打到 ATK-MODULE 侧，接 USB-TTL）
 *    PB10 = TX, PB11 = RX, 115200-8-N-1
 * ========================================================================== */
#define DBG_UART               USART3
#define DBG_UART_IRQn          USART3_IRQn
#define DBG_UART_CLK_ENABLE()  __HAL_RCC_USART3_CLK_ENABLE()

/* ==========================================================================
 * 5. 板上按键与 LED（直接用探索者板载资源，方便脱机操作）
 *    KEY0=PE4  KEY1=PE3  KEY2=PE2  （按下为低）
 *    DS0 =PF9 红灯   DS1 =PF10 绿灯
 * ========================================================================== */
#define KEY0_PORT              GPIOE
#define KEY0_PIN               GPIO_PIN_4
#define KEY1_PORT              GPIOE
#define KEY1_PIN               GPIO_PIN_3
#define KEY2_PORT              GPIOE
#define KEY2_PIN               GPIO_PIN_2

#define LED_R_PORT             GPIOF
#define LED_R_PIN              GPIO_PIN_9
#define LED_G_PORT             GPIOF
#define LED_G_PIN              GPIO_PIN_10

#define KEY_LED_GPIO_CLK_ENABLE() do{ __HAL_RCC_GPIOE_CLK_ENABLE(); \
                                      __HAL_RCC_GPIOF_CLK_ENABLE(); }while(0)

#endif /* __BSP_PINMAP_H */
