#ifndef __MAN_FLASH_STM32F4_H_
#define __MAN_FLASH_STM32F4_H_

#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"

#define KEY_1 0x45670123
#define KEY_2 0xCDEF89AB

#define sector_0 	0UL
#define sector_1 	1UL
#define sector_2 	2UL
#define sector_3 	3UL
#define sector_4 	4UL
#define sector_5 	5UL
#define sector_6 	6UL
#define sector_7 	7UL
#define sector_8 	8UL
#define sector_9 	9UL
#define sector_10 	10UL
#define sector_11 	11UL

#define BYTE        0x00
#define HALFWORD    0x01
#define WORD        0x10
#define DOUBLEWORD  0x11

void flash_Unlock();
void flash_Erase(uint32_t sector);
void flash_Lock();
void flash_programDoubleWord(uint32_t address,uint64_t data);
void flash_programWord(uint32_t address,uint32_t data);
void flash_programHalfWord(uint32_t address,uint16_t data);
void flash_programByte(uint32_t address,uint8_t data);



#endif
