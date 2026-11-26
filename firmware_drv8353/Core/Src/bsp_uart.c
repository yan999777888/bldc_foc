/**
  ******************************************************************************
  * @file    bsp_uart.c
  * @brief   调试串口 + printf 重定向
  * @note    printf 是阻塞发送，115200 下一行 100 字符约 9ms，
  *          只适合 1Hz 级别的打印，不要放进高速控制环里。
  ******************************************************************************
  */

#include "main.h"
#include "bsp_uart.h"
#include <stdio.h>
#include <string.h>

UART_HandleTypeDef huart3;

void UART_Init(void)
{
    huart3.Instance          = DBG_UART;
    huart3.Init.BaudRate     = 115200;
    huart3.Init.WordLength   = UART_WORDLENGTH_8B;
    huart3.Init.StopBits     = UART_STOPBITS_1;
    huart3.Init.Parity       = UART_PARITY_NONE;
    huart3.Init.Mode         = UART_MODE_TX_RX;
    huart3.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart3.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart3) != HAL_OK) {
        Error_Handler();
    }
}

void UART_SendString(const char *s)
{
    HAL_UART_Transmit(&huart3, (uint8_t *)s, (uint16_t)strlen(s), 1000);
}

void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
    if (huart->Instance != DBG_UART) {
        return;
    }

    GPIO_InitTypeDef gpio = {0};
    DBG_UART_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    gpio.Pin       = GPIO_PIN_10 | GPIO_PIN_11;
    gpio.Mode      = GPIO_MODE_AF_PP;
    gpio.Pull      = GPIO_PULLUP;
    gpio.Speed     = GPIO_SPEED_FREQ_HIGH;
    gpio.Alternate = GPIO_AF7_USART3;
    HAL_GPIO_Init(GPIOB, &gpio);
}

/* printf 重定向到 USART3（配合 Keil 的 Use MicroLIB） */
int fputc(int ch, FILE *f)
{
    (void)f;
    uint8_t c = (uint8_t)ch;
    HAL_UART_Transmit(&huart3, &c, 1, 100);
    return ch;
}
