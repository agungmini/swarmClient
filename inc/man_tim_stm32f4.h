#ifndef __MAN_TIM_STM32F4_H
#define __MAN_TIM_STM32F4_H

#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "math.h"
#include "stdlib.h"

#define channel1 0
#define channel2 1
#define channel3 2
#define channel4 3

#define KIRI 0
#define KANAN 1

void TIM4_Init(void);
void TIM5_Init(void);
void TIM6_Init(void);
void TIM3_Init(void);
void generate_PWM(TIM_TypeDef* TIMx,uint8_t channel,uint16_t value);
void motor_jalan(TIM_TypeDef* TIMx,uint8_t motor,int dc_pwm);
void TIM1_Init(void);
void TIM2_Init(void);

#endif
