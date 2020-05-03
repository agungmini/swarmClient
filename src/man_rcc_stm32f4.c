#include <man_rcc_stm32f4.h>

//clock configuration
void SystemClock_Config(void){
	RCC_OscInitTypeDef RCC_OscInitStruct;
	RCC_ClkInitTypeDef RCC_ClkInitStruct;
	RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	RCC_OscInitStruct.OscillatorType= RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState= RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState= RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource= RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM= 8;
	RCC_OscInitStruct.PLL.PLLN= 336;
	RCC_OscInitStruct.PLL.PLLP= RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ= 7;
	if(HAL_RCC_OscConfig(&RCC_OscInitStruct)!= HAL_OK)Error_Handler();

	RCC_ClkInitStruct.ClockType= RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource= RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider= RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider= RCC_HCLK_DIV4;	//42MHz untuk APB1, untuk timer jadi 84MHz
	RCC_ClkInitStruct.APB2CLKDivider= RCC_HCLK_DIV8;	//21MHz	untuk APB2
	if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct,FLASH_LATENCY_5)!= HAL_OK)Error_Handler();

	PeriphClkInitStruct.PeriphClockSelection= RCC_PERIPHCLK_I2S;
	PeriphClkInitStruct.PLLI2S.PLLI2SN= 192;
	PeriphClkInitStruct.PLLI2S.PLLI2SR= 2;
	if(HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct)!= HAL_OK)Error_Handler();

	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_SYSCFG_CLK_ENABLE();
	__HAL_RCC_ADC1_CLK_ENABLE();
	__HAL_RCC_ADC2_CLK_ENABLE();
	__HAL_RCC_I2C1_CLK_ENABLE();
	__HAL_RCC_USART3_CLK_ENABLE();
	__HAL_RCC_USART2_CLK_ENABLE();
	__HAL_RCC_DMA1_CLK_ENABLE();
	__HAL_RCC_TIM4_CLK_ENABLE();
	__HAL_RCC_TIM5_CLK_ENABLE();
	__HAL_RCC_TIM3_CLK_ENABLE();
	__HAL_RCC_TIM2_CLK_ENABLE();
	__HAL_RCC_TIM1_CLK_ENABLE();
	__HAL_RCC_TIM6_CLK_ENABLE();
}

/////////////////////////////////////////////GPIO configuration
void GPIO_Init(void){
	//Port A1 dan A2 sebagai analog input, no pull resistor
	GPIOA->MODER|= (3UL<<2U)|(3UL<<4U);

	//Port D15, D14, D13 dan D12 sebagai output led, push pull, very high speed
	GPIOD->MODER|= (1UL<<30U)|(1UL<<28U)|(1UL<<26U)|(1UL<<24U);
	GPIOD->OSPEEDR|= (3UL<<30U)|(3UL<<28U)|(3UL<<26U)|(3UL<<24U);
	GPIOD->BSRR|= (1UL<<15)|(1UL<<14)|(1UL<<13)|(1UL<<12);

	//Port D0 exti0, D1 exti1, D2 exti2, D3 exti3 sebagai external interrupt falling edge
	SYSCFG->EXTICR[0]|= (3UL)|(3UL<<4U)|(3UL<<8U)|(3UL<<12U);
	EXTI->IMR|= (1UL)|(1UL<<1)|(1UL<<2)|(1UL<<3);
	EXTI->FTSR|= (1UL)|(1UL<<1)|(1UL<<2)|(1UL<<3);

	//Port B10 tx, B11 rx USART3, alternate funct push pull, no pull, very high speed
	GPIOB->MODER|= (2UL<<20U)|(2UL<<22U);
	GPIOB->OSPEEDR|= (3UL<<20U)|(3UL<<22U);
	GPIOB->AFR[1]|= (7UL<<8U)|(7UL<<12U);

	//Port D5 tx, D6 rx USART2, alternate funct push pull, no pull, very high speed
	GPIOD->MODER|= (2UL<<10U)|(2UL<<12U);
	GPIOD->OSPEEDR|= (3UL<<10U)|(3UL<<12U);
	GPIOD->AFR[0]|= (7UL<<20U)|(7UL<<24U);

	//Port B7 sda, B6 scl I2C1, alternate funct open drain, no pull, very high speed
	GPIOB->MODER|=  (2UL<<14U)|(2UL<<12U);
	GPIOB->OSPEEDR|= (3UL<<14U)|(3UL<<12U);
	GPIOB->OTYPER|= (1UL<<7U)|(1UL<<6U);
	GPIOB->AFR[0]|= (4UL<<28U)|(4UL<<24U);

	//Port B0,B1,B4,B5 sebagai TIM3 ch1,ch2,ch3,ch4, alternate funct overdrain, no pull, highspeed
	GPIOB->MODER|= (2UL)|(2UL<<2U)|(2UL<<8U)|(2UL<<10U);
	GPIOB->OTYPER|= (1UL)|(1UL<<1U)|(1UL<<4U)|(1UL<<5U);
	GPIOB->OSPEEDR|= (3UL)|(3UL<<2U)|(3UL<<8U)|(3UL<<10U);
	GPIOB->AFR[0]|= (2UL)|(2UL<<4U)|(2UL<<16U)|(2UL<<20U);

	//Port A8 TIM1 ch1, A15 TIM2 ch1
	GPIOA->MODER|= (2UL<<30U)|(2UL<<16U);
	GPIOA->AFR[1]|= (1UL<<28U)|(1UL);
	//port A8 external interrupt 8 dan port 15 external interrupt 15

}

void Error_Handler(void){

}
