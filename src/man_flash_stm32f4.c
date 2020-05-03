#include <man_flash_stm32f4.h>

void flash_Unlock(){
	if(FLASH->CR & (1UL<<31U)){
		FLASH->KEYR= KEY_1;
		FLASH->KEYR= KEY_2;
	}
}

void flash_Erase(uint32_t sector){
	while((FLASH->SR& (1UL<<16)));	//wait until there is no flash operation
	FLASH->CR&= ~(15UL<<3U);
	FLASH->CR|= (sector<<3U)|(1UL<<1U);
	FLASH->CR|= (1UL<<16U);
	while(FLASH->SR & (1UL<<16U));	//wait until flash operation finishes
}

void flash_Lock(){
	FLASH->CR|= (1UL<<31U);
}

void flash_programDoubleWord(uint32_t address,uint64_t data){
	while(FLASH->SR & (1UL<<16U));
	FLASH->CR|= (1UL);
	FLASH->CR&= ~(3UL<<8U);
	FLASH->CR|= (3UL<<8U);

	*(__IO uint32_t*)address = (uint32_t)data;
	*(__IO uint32_t*)(address+4) = (uint32_t)(data >> 32);
}

void flash_programWord(uint32_t address,uint32_t data){
	while(FLASH->SR & (1UL<<16U));
	FLASH->CR|= (1UL);
	FLASH->CR&= ~(3UL<<8U);
	FLASH->CR|= (2UL<<8U);

	*(__IO uint32_t*)address = (uint32_t)data;
}

void flash_programHalfWord(uint32_t address,uint16_t data){
	while(FLASH->SR & (1UL<<16U));
	FLASH->CR|= (1UL);
	FLASH->CR&= ~(3UL<<8U);
	FLASH->CR|= (1UL<<8U);

	*(__IO uint32_t*)address = (uint16_t)data;
}

void flash_programByte(uint32_t address,uint8_t data){
	while(FLASH->SR & (1UL<<16U));
	FLASH->CR|= (1UL);
	FLASH->CR&= ~(3UL<<8U);

	*(__IO uint32_t*)address = (uint8_t)data;
}
