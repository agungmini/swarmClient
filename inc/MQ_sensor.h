#ifndef __MQ_SENSOR_H
#define __MQ_SENSOR_H

#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "math.h"

float MQ_Get_Resistance(uint16_t value,float RL);
float MQ_Get_PPM(float rs_ro_ratio,float *curve);
float MQ_Calibration(float *data,float RFactor,int size);

#endif
