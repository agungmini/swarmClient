#ifndef FUZZY_H_
#define FUZZY_H_

#ifdef __cplusplus
extern "C"{
#endif

#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"
#include <stdio.h>

//untuk Y
#define J 75
#define M 55
#define D 30
#define S 0

//untuk teta
#define L 5
#define C 4
#define N 3
#define Z 0

int fs_segitiga(float x,float lbound,float max,float hbound,int Val);
int fs_trapesium_samakaki(float x,float lbound,float lmax,float hmax,float hbound,int Val);
int fs_trapesium_sikukiri(float x,float lbound,float hmax,float hbound,int Val);
int fs_trapesium_sikukanan(float x,float lbound,float lmax,float hbound,int Val);
int fs_kotak(int x,int lower,int upper,int Val);
void inference(int *param1,int *param2,int *param3,int *output);
void center_area(int *inference_result,int size,int rule[][63],int *output);
void inference_lidar(int *param1,int *param2,int *param3,int *output);
void center_area_lidar(int pos,int *inference_result,int size,int rule[][2][36],int *output);
void inference_avoidance(int *left,int *right,int *output_inference);
int center_area_avoidance(int *inference_result,int size,int *rule);
void inference_formation(int param[][4],int *output_inference);
int center_area_formation(int *inference_result,int size,int *rule);

#ifdef __cplusplus
}
#endif

#endif /* FUZZY_H_ */
