#ifndef LCD_I2C_H_
#define LCD_I2C_H_

#ifdef __cplusplus
extern "C"{
#endif

#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"
#include <man_i2c_stm32f4.h>
#include "stdlib.h"
#include "stdio.h"
#include "string.h"

/*device address*/
#define PCF8574A	0x7E
#define PCF8574		0x4E

/*mode*/
#define BACKLIGHT 	0x08
#define EN	 		0x04
#define RS			0x01

/*command*/
#define FUNCTSET	0x02
#define MODE4BIT	(FUNCTSET | 0)
#define MODE8BIT	(FUNCTSET | 0x10)
#define MODE1LINE	(FUNCTSET | 0)
#define MODE2LINE	(FUNCTSET | 0x08)
#define MODE5DOTS	(FUNCTSET | 0)
#define MODE11DOTS	(FUNCTSET | 0x04)
#define ENTRYMODE	0x04
#define INC			(ENTRYMODE | 0x02)
#define DEC			(ENTRYMODE | 0)
#define SHFTR		(ENTRYMODE | 0x00)
#define SHFTL		(ENTRYMODE | 0x01)
#define DISPLAY 	0x08
#define DPYON		(DISPLAY | 0x04)
#define DPYOFF		(DISPLAY | 0)
#define CLRDIS		0x01
#define HOME		0x02
#define	DDRAM		0x80

/*fungsi untuk mengatur pin enable LCD
 * i2cx merupakan peripheral I2C yang digunakan untuk transmisi data
 * address merupakan alamat dari I2C GPIO
 * data merupakan data yang hendak ditransmisikan*/
void i2c_lcdEnable(I2C_TypeDef* i2cx,uint8_t address,uint8_t data);

/*fungsi untuk memberikan logika sesuai dengan data 8bit pada pin LCD
 * i2cx merupakan peripheral I2C yang digunakan untuk transmisi data
 * address merupakan alamat dari I2C GPIO
 * data merupakan data yang hendak ditransmisikan
 * mode merupakan mode EN dan RS*/
void i2c_lcdWrite8bit(I2C_TypeDef* i2cx,uint8_t address,uint8_t data,uint8_t mode);

/*fungsi untuk memberikan logika sesuai dengan data 4 bit pada pin lcd*
 * i2cx merupakan peripheral I2C yang digunakan untuk transmisi data
 * address merupakan alamat dari I2C GPIO
 * data merupakan data yang hendak ditransmisikan
 * mode merupakan mode EN dan RS*/
void i2c_lcdWrite4bit(I2C_TypeDef* i2cx,uint8_t address,uint8_t data,uint8_t mode);

/*fungsi untuk memberikan perintah pada LCD
 * i2cx merupakan peripheral I2C yang digunakan untuk transmisi data
 * address merupakan alamat dari I2C GPIO
 * command merupakan perintah yang hendak dikirimkan ke LCD*/
void i2c_lcdCommand(I2C_TypeDef* i2cx,uint8_t address,uint8_t command);

/*menghapus tulisan pada LCD
 * i2cx merupakan peripheral I2C yang digunakan untuk transmisi data
 * address merupakan alamat dari I2C GPIO*/
void i2c_lcdClear(I2C_TypeDef* i2cx,uint8_t address);

/*mengembalikan kursor LCD pada 0,0
 * i2cx merupakan peripheral I2C yang digunakan untuk transmisi data
 * address merupakan alamat dari I2C GPIO*/
void i2c_lcdHome(I2C_TypeDef* i2cx,uint8_t address);

/*menuliskan karakter pada LCD
 * i2cx merupakan peripheral I2C yang digunakan untuk transmisi data
 * address merupakan alamat dari I2C GPIO
 * character merupakan 1 char */
void i2c_lcdWriteChar(I2C_TypeDef* i2cx,uint8_t address,char character);

/*menuliskan string pada LCD
 * i2cx merupakan peripheral I2C yang digunakan untuk transmisi data
 * address merupakan alamat dari I2C GPIO
 * characters merupakan input array of char atau string*/
void i2c_lcdWriteStr(I2C_TypeDef* i2cx,uint8_t address,char *characters);

/*meletakkan kursor LCD
 * i2cx merupakan peripheral I2C yang digunakan untuk transmisi data
 * address merupakan alamat dari I2C GPIO
 * rows baris
 * cols kolom*/
void i2c_lcdSetCursor(I2C_TypeDef* i2cx,uint8_t address,uint8_t rows,uint8_t cols);

/*inisialisasi LCD
 * i2cx merupakan peripheral I2C yang digunakan untuk transmisi data
 * address merupakan alamat dari I2C GPIO*/
void i2c_lcdInit(I2C_TypeDef* i2cx,uint8_t address);

#ifdef __cplusplus
}
#endif

#endif /* LCD_I2C_H_ */
