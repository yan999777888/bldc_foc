/**
  ******************************************************************************
  * @file    bsp_uart.h
  * @brief   调试串口 USART3（PB10=TX / PB11=RX，115200）
  ******************************************************************************
  */

#ifndef __BSP_UART_H
#define __BSP_UART_H

#include "bsp_pinmap.h"

void UART_Init(void);
/* 直接发一段字节，不经过 printf */
void UART_SendString(const char *s);

#endif /* __BSP_UART_H */
