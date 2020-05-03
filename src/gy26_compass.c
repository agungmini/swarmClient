#include <gy26_compass.h>

void reset_gy26(I2C_TypeDef* I2Cx){
	I2C_Start(I2Cx);
	I2C_Addr(I2Cx,GY26_WR);
	I2C_WriteData(I2Cx,PRELIMINARY);
	I2C_WriteData(I2Cx,RETURN_FACTORY0);
	I2C_WriteData(I2Cx,RETURN_FACTORY1);
	I2C_WriteData(I2Cx,RETURN_FACTORY2);
	I2C_WriteData(I2Cx,RETURN_FACTORY3);
	I2C_Stop(I2Cx);
}

int get_angle(I2C_TypeDef* I2Cx){
	uint8_t tmp1,tmp2,tmp3,tmp4,tmp5,tmp6,tmp7,tmp8;

	I2C_Start(I2Cx);
	I2C_Addr(I2Cx,GY26_WR);
	I2C_WriteData(I2Cx,PRELIMINARY);
	I2C_WriteData(I2Cx,MEASURE_ANGLE);
	I2C_Stop(I2Cx);
	I2C_Start(I2Cx);
	I2C_Addr(I2Cx,GY26_RD);
	tmp1= I2C_getval(I2Cx,ACK);
	tmp2= I2C_getval(I2Cx,ACK);
	tmp3= I2C_getval(I2Cx,ACK);
	tmp4= I2C_getval(I2Cx,ACK);
	tmp5= I2C_getval(I2Cx,ACK);
	tmp6= I2C_getval(I2Cx,ACK);
	tmp7= I2C_getval(I2Cx,ACK);
	tmp8= I2C_getval(I2Cx,NACK);

	return ((tmp2*257+tmp3)/10);
}
