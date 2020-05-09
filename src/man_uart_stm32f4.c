#include <man_uart_stm32f4.h>

//////////////////////////////////////////////UART configuration
void UART_Init(void){
	//uart3, APB1 42MHz
	USART3->BRR= (12UL)|(22UL<<4U);	//baudrate 115200
	USART3->CR3|= (1UL<<6U); //DMA reception
	USART3->CR1|= (1UL<<13U); //8 bit 1 stop bit

	//uart2, APB1 42MHz
	USART2->BRR= (12UL)|(22UL<<4U); //baudrate 115200
	USART2->CR1|= (1UL<<13U); //8 bit 1 stop bit
}

void UART2_InterruptReception(void){
	USART2->CR1|= (1UL<<5U);

	NVIC->ISER[1]|= (1UL<<6U);
	NVIC->IP[9]|= (15UL<<16U);	//interrupt priority to 15
}

void UART_sendChar(USART_TypeDef* USARTx,const char tmp){
	USARTx->CR1|= (1UL<<3U);
	while(!(USARTx->SR& (1UL<<7U))); //awalnya ~(1UL<<7U)
	USARTx->DR= tmp;
}

void UART_sendStr(USART_TypeDef* USARTx,const char *ptr){
	while(*ptr)UART_sendChar(USARTx,*ptr++);
}

char UART_getChar(USART_TypeDef* USARTx){
	USARTx->CR1|= (1UL<<2U);	//enable reception
	while(!(USARTx->SR& (1UL<<5U))); //wait until RXNE set
	return USARTx->DR;
}

void UART_Reception(USART_TypeDef* USARTx,char *buff){

}

void UART_DMA_reception(USART_TypeDef* USARTx){
	USARTx->CR1|= (1UL<<2U);	//enable reception
}

////////////////////////////////////////////////DMA1 Stream 1 channel 4 configuration untuk USART3 RX
void DMA1_Stream1_Init(uint32_t memory_address,uint32_t peripheral_address,uint16_t size){
	DMA1_Stream1->CR|= (4UL<<25U)|(3UL<<16U)|(1UL<<10U)|(1UL<<8U)|(1UL<<4U); //channel 4,priority level very high,circular mode,transfer complete interrupt
	DMA1_Stream1->NDTR= size;
	DMA1_Stream1->M0AR= memory_address;
	DMA1_Stream1->PAR= peripheral_address;
	DMA1_Stream1->CR|= (1UL);

	//DMA1 Stream 1 interrupt
	NVIC->ISER[0]|= (1UL<<12U);
	NVIC->IP[3]|= (1UL);	//priority interrupt 1
}

