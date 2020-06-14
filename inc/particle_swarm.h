#ifndef __PARTICLE_SWARM_H
#define __PARTICLE_SWARM_H

//#include <esp01_AT.h>
//#include <man_uart_stm32f4.h>
//#include "string.h"
//#include "stdlib.h"
//#include "stdio.h"

#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "math.h"

#define PI 57.3

int metaheuristicGetpos(int orient,int *fitness,int *pos1,int *pos2);

#endif
