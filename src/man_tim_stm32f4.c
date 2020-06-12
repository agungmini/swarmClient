#include <man_tim_stm32f4.h>

void TIM4_Init(void){
	TIM4->CR1|= (1UL<<7U)|(1UL<<2U); //no prescale, ARR buffered,count up,overflow generate interrupt
	TIM4->DIER|= (1UL);	//update interrupt enable
	TIM4->PSC= (4000UL); //prescale 84MHz to 5Hz as fs gas sensing
	TIM4->ARR= (4100UL);
	TIM4->CR1|= (1UL); //counter enable

	//set interrupt untuk timer4
	//NVIC->ISPR[0]|= (1UL<<30U); //set pending interrupt
	NVIC->IP[7]|= (8UL<<16U);	//set priority 8
	NVIC->ISER[0]|= (1UL<<30U);	//enable NVIC for timer4 global interrupt
}

void TIM5_Init(void){
	TIM5->CR1|= (1UL<<7U)|(1UL<<4U)|(1UL<<2U); //no prescale, ARR buffered,count down,overflow generate interrupt
	TIM5->DIER|= (1UL); //update interrupt enable
	TIM5->PSC= (10000UL); //prescale 84MHz to 1Hz
	TIM5->ARR= (8400UL);
	TIM5->CR1|= (1UL); //counter enable

	//set interrupt untuk timer5
	//NVIC->ISPR[1]|= (1UL<<(50%32));
	NVIC->IP[12]|= (9UL<<16U);
	NVIC->ISER[1]|= (1UL<<(50%32));
}

void TIM6_Init(void){
	TIM6->CR1|= (1UL<<7U)|(1UL<<2U);
	TIM6->DIER|= (1UL);
	TIM6->PSC= (1000UL);	//prescale menjadi 250Hz
	TIM6->ARR= (336UL);
	//TIM6->CR1|= (1UL);

	//set interrupt untuk timer 6
	//NVIC->ISPR[1]|= (1UL<<(54%32));
	NVIC->IP[13]|= (10UL<<5U); //interrupt priority to 10
	NVIC->ISER[1]|= (1UL<<(54%32));
}

void TIM3_Init(void){
	TIM3->CR1|= (1UL<<7U)|(2UL<<8U); //prescaled by 4 (21MHz),ARR buffered,update generation by setting UG bit
	TIM3->PSC= (105UL); //di prescale sehingga menjadi sekitaran 20KHz
	TIM3->ARR= (1000UL);	//duty cycle dari 0% hingga 100%

	//channel 1-4 configuration
	TIM3->CCMR1|= (1UL<<2U)|(1UL<<3U)|(6UL<<4U); //channel 1, PWM mode 1, output compare fast,preload enable (set UG when CCR1 changed)
	TIM3->CCMR1|= (1UL<<10U)|(1UL<<11U)|(6UL<<12U); //channel 2, PWM mode 1, output compare fast,preload enable (set UG when CCR1 changed)
	TIM3->CCMR2|= (1UL<<2U)|(1UL<<3U)|(6UL<<4U); //channel 3, PWM mode 1, output compare fast,preload enable (set UG when CCR1 changed)
	TIM3->CCMR2|= (1UL<<10U)|(1UL<<11U)|(6UL<<12U); //channel 4, PWM mode 1, output compare fast,preload enable (set UG when CCR1 changed)

	TIM3->CCER|= (1UL)|(1UL<<4U)|(1UL<<8U)|(1UL<<12U);

	//TIM3->CR1|= (1UL);	//counter enable
}

void generate_PWM(TIM_TypeDef* TIMx,uint8_t channel,uint16_t value){
	switch(channel){
	case channel1: TIMx->CCR1= value;
		break;
	case channel2: TIMx->CCR2= value;
		break;
	case channel3: TIMx->CCR3= value;
		break;
	case channel4: TIMx->CCR4= value;
		break;
	}
	TIMx->EGR|= (1UL);
}

void motor_jalan(TIM_TypeDef* TIMx,uint8_t motor,int dc_pwm){
	if(motor==KIRI){
		if(dc_pwm<= 0){
			generate_PWM(TIMx,channel1,abs(dc_pwm));
			generate_PWM(TIMx,channel2,0);
		}
		else{
			generate_PWM(TIMx,channel2,abs(dc_pwm));
			generate_PWM(TIMx,channel1,0);
		}
	}
	else{
		if(dc_pwm<= 0){
			generate_PWM(TIMx,channel4,0);
			generate_PWM(TIMx,channel3,abs(dc_pwm));
		}
		else{
			generate_PWM(TIMx,channel3,0);
			generate_PWM(TIMx,channel4,abs(dc_pwm));
		}
	}
}

void TIM1_Init(void){
	TIM1->CR1|= (1UL<<7U)|(1UL<<1U);
	TIM1->CR2|= (1UL<<7U);

	TIM1->SMCR|= (4UL<<4U)|(7UL);
	TIM1->CCMR1|= (15UL<<4U)|(1UL);
	TIM1->CR1|= (1UL);
}

void TIM2_Init(void){
	TIM2->CR1|= (1UL<<7U)|(1UL<<1U);
	TIM2->CR2|= (1UL<<7U);

	TIM2->SMCR|= (4UL<<4U)|(7UL);
	TIM2->CCMR1|= (15UL<<4U)|(1UL);
	TIM2->CR1|= (1UL);
}
