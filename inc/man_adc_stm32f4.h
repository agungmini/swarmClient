#ifndef __MAN_ADC_STM32F4_H
#define __MAN_ADC_STM32F4_H

#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"

void ADC_Init(void);
uint16_t ADC_getVal(ADC_TypeDef* ADCx);
void ADC_getValSimultaneous(ADC_Common_TypeDef* ADCc,ADC_TypeDef* ADCx1,ADC_TypeDef* ADCx2,uint16_t* buff);

#endif
