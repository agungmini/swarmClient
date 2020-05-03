#ifndef __MAN_I2C_STM32F4_H
#define __MAN_I2C_STM32F4_H

#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "stdlib.h"
#include "stdio.h"
#include "string.h"
#include <dwt_stm32_delay.h>

#define ACK 1
#define NACK 0

void I2C1_Init(void);
void I2C_Start(I2C_TypeDef* I2Cx);
void I2C_Addr(I2C_TypeDef* I2Cx,uint8_t address);
void I2C_WriteData(I2C_TypeDef* I2Cx,uint8_t tmp);
void I2C_Stop(I2C_TypeDef* I2Cx);
void I2C_Transmit(I2C_TypeDef* I2Cx,uint8_t ADDR,uint8_t tmp);
void I2C_TransmitPage(I2C_TypeDef* I2Cx,uint8_t ADDR,uint8_t *ptr);
uint8_t I2C_getval(I2C_TypeDef* I2Cx,uint32_t acknowledgement);

#endif
