#include "MQ_sensor.h"

float MQ_Get_Resistance(uint16_t value,float RL){
	float temp= value*1.18;
	return ((RL*5000/temp))-RL;
}

float MQ_Get_PPM(float rs_ro_ratio,float *curve){
	return (float)pow(10,((rs_ro_ratio-curve[1])/curve[0])+curve[2]);
}

float MQ_Calibration(float *data,float RFactor,int size){
	int temp= 0;
	for(int i=0;i<size;i++){
		temp= temp+data[i];
	}
	temp=temp/size;
	return (float)temp/RFactor;
}
