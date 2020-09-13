#ifndef __PARTICLE_SWARM_H
#define __PARTICLE_SWARM_H

#include <esp01_AT.h>
#include <man_uart_stm32f4.h>
#include "string.h"
#include "stdlib.h"
#include "stdio.h"

#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "math.h"

#define PI 57.3

/*fungsi untuk mendapatkan koordinat kartesian tujuan gerak robot
 * ida id robot untuk proses debugging
 * ID id robot untuk proses debugging
 * orient merupakan orientasi dari robot
 * fitness merupakan array nilai fitness dari seluruh robot
 * direction merupakan array berisi arah gerak masing-masing robot dalam kawanan
 * pos1 meruapakn array berisi koordinat polar sudut posisi masing-masing robot
 * pos2 merupakan array berisi koordinat polar besar jarak masing-masing robot
 * form_shape merupakan bentuk dari formasi. terdiri atas jarak antar robot dan sudut yang dibentuk
 * best merupakan koordinat kartesian posisi robot yang paling depan
 * cartDest merupakan keluaran berupa jarak x dan y dari posisi robot (reference) menuju posisi tujuan*/
int metaheuristicGetpos_and_cartDest(char ida,int ide,int orient,int *fitness,int *direction,int *pos1,int *pos2,int *form_shape,int *best,int *cartDest);

/*fungsi untuk mendapatkan koordinat kartesian tujuan gerak robot
 * fungsi ini digunakan untuk metode tanpa metaheuristik
 * posisi merupakan posisi robot didalam formasi
 * orient merupakan orientasi robot
 * form shape merupakan bentuk dari formasi. terdiri atas jarak antar robot dan sudut segitiga yang dibentuk
 * bestPos merupakan koordinat kartesian posisi robot yang paling depan
 * cartDist merupakan keluaran berupa jarak x dan y dari posisi robot menuju posisi tujuan*/
void get_cartDest(int posisi,int orient,int direct,int *form_shape,int *bestPos,int *cartDist);

#endif
