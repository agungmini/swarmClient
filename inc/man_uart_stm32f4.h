#ifndef __MAN_UART_STM32F4_H
#define __MAN_UART_STM32F4_H

#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"

//UART
void UART_Init(void);
void UART2_InterruptReception(void);
void UART_sendStr(USART_TypeDef* USARTx,const char *ptr);
void UART_Reception(USART_TypeDef* USARTx,char *buff);
char UART_getChar(USART_TypeDef* USARTx);
void UART_DMA_reception(USART_TypeDef* USARTx);

//DMA2
void DMA1_Stream1_Init(uint32_t memory_address,uint32_t peripheral_address,uint16_t size);

#endif
