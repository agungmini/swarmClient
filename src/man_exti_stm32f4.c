#include <man_exti_stm32f4.h>

void EXTI_Init(){
	SYSCFG->EXTICR[0]|= (3UL)|(3UL<<4U)|(3UL<<8U)|(3UL<<12U); //external interrupt ch0=>PD,ch1=>PD,ch2=>PD,ch3=>PD
	EXTI->FTSR|= (1UL)|(2UL)|(4UL)|(8UL);	//semua channel falling edge
	//enable
	NVIC->ISER[0]|= (1UL<<6U)|(1UL<<7U)|(1UL<<8U)|(1UL<<9U);	//set enable nvic untuk ch0,ch1,ch2 dan ch3
	NVIC->IP[1]|= (11UL<<16U)|(12UL<<24U);
	NVIC->IP[2]|= (13UL)|(14UL<<8U);
}
