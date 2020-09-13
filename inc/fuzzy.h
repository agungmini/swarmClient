#ifndef FUZZY_H_
#define FUZZY_H_

#ifdef __cplusplus
extern "C"{
#endif

#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"
#include <stdio.h>

/*fuzzy rule based*/
/*untuk Y*/
#define J 75
#define M 55
#define D 30
#define S 0

/*untuk teta*/
#define L 6
#define C 4
#define N 2
#define Z 0

/*fungsi derajat keanggotaan f(x)= x
 * terdiri atas fungsi segitiga, trapesium sama kaki, trapesium siku kanan, trapesium siki kiri
 * x merupakan nilai crisp input
 * lbound merupakan nilai x bawah pada f(x)=0
 * nilai x pada saat f(fx)=max
 * hbpund merupakan nilai x atas pada f(x)=0
 * lmax merupakan nilai x bawah pada f(x)= max
 * hmax merupakan nilai x atas pada f(x)= max
 * val merupakan nilai maksimal fungsi f(x)*/
int fs_segitiga(float x,float lbound,float max,float hbound,int Val);
int fs_trapesium_samakaki(float x,float lbound,float lmax,float hmax,float hbound,int Val);
int fs_trapesium_sikukiri(float x,float lbound,float hmax,float hbound,int Val);
int fs_trapesium_sikukanan(float x,float lbound,float lmax,float hbound,int Val);

/*merupakan fungsi inference dari fuzzy input
 * param1 berisi array fuzzy input berupa resultan konsentrasi gas
 * param2 berisi array fuzzy input berupa perubahan konsentrasi gas
 * param3 berisi array fuzzy input berupa selisih perubahan konsentrasi gas pada sensor stereo
 * output merupakan array hasil inference*/
void inference(int *param1,int *param2,int *param3,int *output);

/*merupakan fungsi inference dari fuzzy input
 * param1 berisi array fuzzy input jarak
 * param2 berisi array fuzzy input perubahan jarak
 * param3 berisi array fuzzy input arah
 * param4 berisi array fuzzy input perubahan arah
 * output merupakan array hasil inference*/
void inference_lidar1(int *param1,int *param2,int *param3,int *param4,int *output);

/*merupakan fungsi inference dari fuzzy input
 * left berisi array fuzzy input jarak pembacaan sensor kiri
 * right berisi array fuzzy input jarak pembacaan sensor kanan*/
void inference_avoidance(int *left,int *right,int *output_inference);

/*center area merupakan fungsi defuzzyfikasi dengan metode center of area
 * inference result merupakan array output dari fungsi inderence
 * size merupakan ukuran array inference result
 * rule merupakan fuzzy rule
 * output merupakan crisp output dari fuzzyfikasi*/
void center_area(int *inference_result,int size,int rule[][63],int *output);

/*center area merupakan fungsi defuzzyfikasi dengan metode center of area
 * inference result merupakan array output dari fungsi inderence lidar1
 * size merupakan ukuran array inference result
 * rule merupakan fuzzy rule
 * output merupakan crisp output dari fuzzyfikasi*/
void center_area_lidar1(int *inference_result,int size,int rule[][81],int *output);

/*center area merupakan fungsi defuzzyfikasi dengan metode center of area
 * inference result merupakan array output dari fungsi inderence avoidance
 * size merupakan ukuran array inference result
 * rule merupakan fuzzy rule*/
int center_area_avoidance(int *inference_result,int size,int *rule);

#ifdef __cplusplus
}
#endif

#endif /* FUZZY_H_ */
