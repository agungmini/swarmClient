#include <man_adc_stm32f4.h>

void ADC_Init(void){
	//channel 1 untuk adc1 dengan 15 cycle;
	ADC1->CR1|= (1UL<<11U);
	ADC1->CR2|= (1UL<<10U)|(1UL);
	ADC1->SQR3|= (1UL);

	//channel 2 untuk adc2 dengan 15 cycle;
	ADC2->CR1|= (1UL<<11U);
	ADC2->CR2|= (1UL<<10U)|(1UL);
	ADC2->SQR3|= (2UL);

	//simultan ADC
	//ADC->CCR|= (6UL); //adc1 dan adc2 simultan
}

uint16_t ADC_getVal(ADC_TypeDef* ADCx){
	ADCx->CR2|= (1UL<<30U);
	while(!(ADCx->SR&(1UL<<1U)));
	ADCx->SR&= ~(1UL<<4U);
	return ADCx->DR&0x00000FFF;
}

void ADC_getValSimultaneous(ADC_Common_TypeDef* ADCc,ADC_TypeDef* ADCx1,ADC_TypeDef* ADCx2,uint16_t* buff){
	ADCx1->CR2|= (1UL<<30U);
	//ADCx2->CR2|= (1UL<<30U);
	while(!((ADCc->CSR&(1UL<<1U))&(ADCc->CSR&(1UL<<9U))));
	buff[0]= ADCc->CDR& 0xFFF;
	buff[1]= (ADCc->CDR>>16)& 0xFFF;
}
