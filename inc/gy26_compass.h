#ifndef _GY26_COMPASS_H_
#define _GY26_COMPASS_H_

#ifdef __cplusplus
extern "C"{
#endif

#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"
#include <man_i2c_stm32f4.h>
#include "stdlib.h"
#include "stdio.h"
#include "string.h"

#define GY26_WR 0xE0
#define GY26_RD 0xE1

#define PRELIMINARY 0x00
#define MEASURE_ANGLE 0x31
#define START_CALIBRATION 0xC0
#define END_CALIBRATION 0xC1
#define RETURN_FACTORY0 0xA0
#define RETURN_FACTORY1 0xAA
#define RETURN_FACTORY2 0xA5
#define RETURN_FACTORY3 0xC5
#define CHANGE_ADDRESS0 0xC1
#define CHANGE_ADDRESS1 0xA0
#define CHANGE_ADDRESS2 0xAA
#define DECL_HIGH 0x03
#define DECL_LOW 0xA4

void reset_gy26(I2C_TypeDef* I2Cx);
int get_angle(I2C_TypeDef* I2Cx);

#ifdef __cplusplus
}
#endif

#endif /* LCD_I2C_H_ */
