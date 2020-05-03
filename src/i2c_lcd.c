#include <i2c_lcd.h>

void i2c_lcdEnable(I2C_TypeDef* i2cx,uint8_t address,uint8_t data){
	uint8_t enable= data|EN;
	uint8_t enableBAR= data & ~EN;
	I2C_Transmit(i2cx,address,enable);
	DWT_Delay_us(4);
	//HAL_Delay(1);
	I2C_Transmit(i2cx,address,enableBAR);
	DWT_Delay_us(4);
	//HAL_Delay(1);
}

void i2c_lcdWrite8bit(I2C_TypeDef* i2cx,uint8_t address,uint8_t data,uint8_t mode){
	uint8_t temp= data|mode|BACKLIGHT;
	I2C_Transmit(i2cx,address,temp);
	i2c_lcdEnable(i2cx,address,data|mode|BACKLIGHT);
}

void i2c_lcdWrite4bit(I2C_TypeDef* i2cx,uint8_t address,uint8_t data,uint8_t mode){
	uint8_t hi_nible,lo_nible;
	hi_nible= data& 0xF0;
	lo_nible= (data & 0x0F)<<4;

	i2c_lcdWrite8bit(i2cx,address,hi_nible,mode);
	i2c_lcdWrite8bit(i2cx,address,lo_nible,mode);
}

void i2c_lcdCommand(I2C_TypeDef* i2cx,uint8_t address,uint8_t command){
	i2c_lcdWrite4bit(i2cx,address,command,0);
	DWT_Delay_us(96);
	//HAL_Delay(1);
}

void i2c_lcdClear(I2C_TypeDef* i2cx,uint8_t address){
	i2c_lcdCommand(i2cx,address,CLRDIS);
	DWT_Delay_us(2000);
	HAL_Delay(1);
}

void i2c_lcdHome(I2C_TypeDef* i2cx,uint8_t address){
	i2c_lcdCommand(i2cx,address,HOME);
	DWT_Delay_us(2000);
	//HAL_Delay(1);
}

void i2c_lcdWriteChar(I2C_TypeDef* i2cx,uint8_t address,char character){
	i2c_lcdWrite4bit(i2cx,address,character,RS);
	DWT_Delay_us(96);
	//HAL_Delay(1);
}

void i2c_lcdWriteStr(I2C_TypeDef* i2cx,uint8_t address,char *characters){
	int i=0;
	while(*characters)i2c_lcdWriteChar(i2cx,address,*characters++);
}

void i2c_lcdSetCursor(I2C_TypeDef* i2cx,uint8_t address,uint8_t rows,uint8_t cols){
	switch(rows){
		case 0: i2c_lcdCommand(i2cx,address,DDRAM|0x00|cols);
		break;
		case 1: i2c_lcdCommand(i2cx,address,DDRAM|0x40|cols);
		break;
		case 2: i2c_lcdCommand(i2cx,address,DDRAM|0x14|cols);
		break;
		case 3: i2c_lcdCommand(i2cx,address,DDRAM|0x54|cols);
		break;
	}
}

void i2c_lcdInit(I2C_TypeDef* i2cx,uint8_t address){
	DWT_Delay_Init();
	DWT_Delay_us(20000);
	//HAL_Delay(1);
	i2c_lcdWrite8bit(i2cx,address,0x30,0);
	DWT_Delay_us(5500);
	//HAL_Delay(1);
	i2c_lcdWrite8bit(i2cx,address,0x30,0);
	DWT_Delay_us(205);
	//HAL_Delay(1);
	i2c_lcdWrite8bit(i2cx,address,0x30,0);

	//HAL_Delay(1);
	i2c_lcdWrite8bit(i2cx,address,0x20,0);
	//HAL_Delay(1);
	i2c_lcdCommand(i2cx,address,MODE4BIT|MODE2LINE|MODE5DOTS);
	i2c_lcdCommand(i2cx,address,DPYOFF);
	i2c_lcdClear(i2cx,address);
	i2c_lcdCommand(i2cx,address,INC);
	i2c_lcdCommand(i2cx,address,DPYON);
}
