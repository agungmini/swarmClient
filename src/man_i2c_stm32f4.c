#include <man_i2c_stm32f4.h>

void I2C1_Init(void){
	I2C1->CR2|= (42UL);
	I2C1->CCR|= (100UL);
	I2C1->TRISE|= (20UL);
	I2C1->CR1|= (1UL);
}

void I2C_Start(I2C_TypeDef* I2Cx){
	while(I2Cx->SR2 & (1UL<<1));
	I2Cx->CR1|= (1UL<<8U); //start generation
	while(!(I2Cx->SR1& (1UL))); //wait until start bit generated 1
}

void I2C_Addr(I2C_TypeDef* I2Cx,uint8_t address){
	I2Cx->DR= address; //write to data register
	while(!(I2Cx->SR1& (1UL<<1U)));
	I2Cx->SR2; //clear addr
}

void I2C_WriteData(I2C_TypeDef* I2Cx,uint8_t tmp){
	while(!(I2Cx->SR1& (1UL<<7U)));
	I2Cx->DR= tmp;
}

void I2C_Stop(I2C_TypeDef* I2Cx){
	while((!(I2Cx->SR1&(1UL<<7U))));
	I2Cx->CR1|= (1UL<<9);
}

void I2C_Transmit(I2C_TypeDef* I2Cx,uint8_t ADDR,uint8_t tmp){
	I2C_Start(I2Cx);
	I2C_Addr(I2Cx,ADDR);
	I2C_WriteData(I2Cx,tmp);
	I2C_Stop(I2Cx);
}

void I2C_TransmitPage(I2C_TypeDef* I2Cx,uint8_t ADDR,uint8_t *ptr){
	I2C_Start(I2Cx);
	I2C_Addr(I2Cx,ADDR);
	while(*ptr)I2C_WriteData(I2Cx,*ptr++);
	I2C_Stop(I2Cx);
}

uint8_t I2C_getval(I2C_TypeDef* I2Cx,uint32_t acknowledgement){
	if(acknowledgement== ACK){
		I2Cx->CR1|= (1UL<<10U);
	}
	else{
		I2Cx->CR1&= ~(1UL<<10U);
		I2Cx->CR1|= (1UL<<9);
	}
	while(!(I2Cx->SR1& (1UL<<6U)));
	return I2Cx->DR;
}
