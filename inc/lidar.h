#ifndef __LIDAR_H_
#define __LIDAR_H_

#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"
#include "math.h"
#include "stdio.h"
#include "stdlib.h"
#include "man_uart_stm32f4.h"

#define PI 57.3

#define YDLIDAR 0
#define RPLIDAR 1

#define YDLIDAR_RATE 128000
#define RPLIDAR_RATE 115200

#define LIDARCOMMAND 0xA5

//untuk ydlidar
#define YD_START 0x60
#define YD_INFO 0x90
#define YD_STOP 0x65
#define YD_HEALTH 0x91

//untuk RPLIDAR
#define RP_START 0x20
#define RP_INFO 0x50
#define RP_STOP 0x25
#define RP_HEALTH 0x52

void lidar_command(USART_TypeDef* UARTx,uint8_t command);
void rplidar_conversion(uint8_t *data_in,int data_inSize,uint16_t limit,uint16_t *data_out,int state);
void probabilistic_filter(uint16_t *input,uint16_t *output,float Gain,int sample,int overlap);
void derivate_graph(uint16_t *input,int *output,int size,int* stdev,int* mean);
void find_robot(int *derivative,uint16_t *distance,int stdev,int mean,int size,float gain,int* angle,uint16_t *dist);
void find_theBest(uint16_t *input,int *input1,int size,int *measured_angle,int *measured_dist,int *out,uint16_t *out1,int limit_distance,int state);
void measure_velocity(int *pos,int *pos1,int orientasi,int *angle,int *angle1,int *velocity,int *direction,int state);
void convert_cartesian(uint16_t *dist,int *angle,int *y_axis,int *x_axis);
uint16_t get_distance(uint16_t *lidar_scan,int start,int end);

#endif
