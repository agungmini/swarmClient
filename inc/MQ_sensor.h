#ifndef __MQ_SENSOR_H
#define __MQ_SENSOR_H

#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "math.h"

/*fungsi ini untuk mendapatkan nilai resistansi Rs dari sensor gas
 * value merupakan nilai tegangan ADC terukur dari sensor
 * RL merupakan resistansi load yang berada pada sensor*/
float MQ_Get_Resistance(uint16_t value,float RL);

/*fungsi ini untuk mendapatkan nilai konsentrasi gas dalam satuan ppm
 * rs_rs_ratio merupakan pembagian antara nilai Rs dengab Ro (konstanta)
 * curve merupakan kurva kalibrasi dari datasheet sensor gas*/
float MQ_Get_PPM(float rs_ro_ratio,float *curve);

/*fungsi ini utuk mendapatkan nilai Ro dari sensor. Ro merupakan konstanta kalibrasi sensor
 * data merupakan array nilai resistansi Rs
 * RFactor merupakan rasio pengukuran pada ppm 1000 N2
 * size merupakan panjang array data*/
float MQ_Calibration(float *data,float RFactor,int size);

#endif
