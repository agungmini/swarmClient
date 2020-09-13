#ifndef __LIDAR_H_
#define __LIDAR_H_

#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"
#include "math.h"
#include "stdio.h"
#include "stdlib.h"
#include "man_uart_stm32f4.h"

#define PI 57.3

/*pemilihan device yang digunakan*/
#define YDLIDAR 0
#define RPLIDAR 1

/*baudrate device*/
#define YDLIDAR_RATE 128000
#define RPLIDAR_RATE 115200

#define LIDARCOMMAND 0xA5

/*command untuk YD LiDAR,
 * YD_START untuk memberikan perintah scanning LiDAR
 * YD_INFO untuk mengetahui status dari LiDAR
 * YD_STOP untuk memberikan perintah LiDAR untuk berhenti scanning
 * YD_HEALTH untuk mengetahui status LiDAR*/
#define YD_START 0x60
#define YD_INFO 0x90
#define YD_STOP 0x65
#define YD_HEALTH 0x91

/*command untuk RP LiDAR,
 * RP_START untuk memberikan perintah scanning LiDAR
 * RP_INFO untuk mengetahui status dari LiDAR
 * RP_STOP untuk memberikan perintah LiDAR untuk berhenti scanning
 * RP_HEALTH untuk mengetahui status LiDAR*/
#define RP_START 0x20
#define RP_INFO 0x50
#define RP_STOP 0x25
#define RP_HEALTH 0x52

/*lidar command merupakan fungsi untuk memberikan perintah dasar kepada LiDAR
 * ex: lidar_command(USART2,RP_STOP), mengirimkan perintah stop melalui peripheral USART2
 * UARTx merupakan peripheral USART yang digunakan
 * command merupakan perintah yang dikirim ke LiDAR*/
void lidar_command(USART_TypeDef* UARTx,uint8_t command);

/*fungsi dibawah ini digunakan untuk mengolah data mentah dari LiDAR menjadi jarak untuk setiap sudut
 * hasil merupakan array integer dengan lebar 360. hasil pengukuran merupakan jarak dalam satuan mm.
 * data_in merupakan array dengan panjang 360 berisi data mentah pengukuran LiDAR
 * data_inSize merupakan lebar dari array data_in
 * limit merupakan nilai tertinggi pengukuran LiDAR
 * data_out merupakan hasil keluaran yang merupakan array mm jarak pengukuran LiDAR
 * state merupakan kondisi inisial LiDAR (mula-mula 0) setelah 1 kali perioda pengukuran, state diubah nilai menjadi 1
 * */
void rplidar_conversion(uint8_t *data_in,int data_inSize,uint16_t limit,uint16_t *data_out,int state);

/*probabilistik filter merupakan filter yang digunakan untuk memperbaiki array pengukuran oleh LiDAR.
 * input merupakan array input hasil konversi
 * output merupakan array keluaran hasil filter
 * Gain untuk melebarkan standar deviasi
 * sample merupakan lebar array yang akan difilter
 * overlap merupakan besar pergesaran untuk melakukan filter*/
void probabilistic_filter(uint16_t *input,uint16_t *output,float Gain,int sample,int overlap);

/*derivate graph merupakan fungsi untuk mendapatkan turunan pertama dari hasil pengukuran LiDAR
 * input merupakan array dari hasil filter
 * output merupakan array turunan pertama dari input
 * size adalah lebar array input dan output
 * stdev adalah besar nilai standar deviasi dari anggota output
 * mean merupakan nilai rata-rata dari anggota output*/
void derivate_graph(uint16_t *input,int *output,int size,int* stdev,int* mean);

/*merupakan fungsi untuk mendapatkan koordinat polar robot lainnya menggunakan lidar.
 * derivatid adalah array yang merupakan array turunan pertama dari hasil scanning lidar
 * distance merupakan array yang berisi hasil scan lidar yang telah mengalami filtrasi
 * stdev merupakan nilai standar deviasi dari anggota derivativ
 * mean merupakan rata-rata nilai dari derivatif
 * size merupakan ukuran minimal dari robot yang hendak dideteksi
 * gain merupakan pelebaran nilai standar deviasi untuk pendeteksian
 * angle merupakan array sudut robot terdeteksi
 * dist merupakan array jarak robot terdeteksi
 * */
void find_robot(int *derivative,uint16_t *distance,int stdev,int mean,int size,float gain,int* angle,uint16_t *dist);

/*fungsi untuk mengeliminasi menjadi 2 robot sisa
 * input merupakan array berisi posisi sudut robot terdeteksi
 * input1 merupakan array berisi posisi jarak robot terdeteksi
 * size merupakan jumlah robot yang terdeteksi
 * measured angle merupakan posisi sudut robot terdeteksi pada waktu t(n-1)
 * measured dist merupakan posisi jarak robot terdeteksi pada watu t(n-1)
 * out adalah 2 posisi sudut robot terbaik
 * out1 adalah 2 posisi jarak robot terbaik
 * limit distance adalah range maksimal robot untuk mendeteksi
 * state merupakan kondisi inisial LiDAR (mula-mula 0) setelah 1 kali perioda pengukuran, state diubah nilai menjadi 1
 * */
void find_theBest(uint16_t *input,int *input1,int size,int *measured_angle,int *measured_dist,int *out,uint16_t *out1,int limit_distance,int state);

/*merupakan fungsi untuk mengukur kecepatan dan arah gerak dari kawanan robot
 * pos merupakan array jarak robot
 * angle merupakan array sudut robot
 * pos1 merupakan array jarak robot pada t(n-1)
 * angle1 merupakan array sudut robot pada t(n-1)
 * orientasi merupakan orientasi robot
 * velocity merupakan array kecepatan robot
 * direction metupkan array kluaran. nilai merupkan arah gerak robot
 * state merupakan kondisi inisial LiDAR (mula-mula 0) setelah 1 kali perioda pengukuran, state diubah nilai menjadi 1
 * */
void measure_velocity(int *pos,int *pos1,int orientasi,int *angle,int *angle1,int *velocity,int *direction,int state);

/*merupakan fungsi untuk merubah koordinat polar menjadi kartesian
 * koordinat polar terdiri dari jarak dan arah, sedangkan dalam koordinat kartesian elemen terdiri atas nilai pada x axis dan y axis
 * */
void convert_cartesian(uint16_t *dist,int *angle,int *y_axis,int *x_axis);

/*fungsi ini digunakan untuk obstacle avoidance
 * lidar scan merupakan hasil scanning lidar berisi array dari jarak
 * start merupakan sudut awal pengukuran,
 * end merupakan sudut akhir pengukuran*/
uint16_t get_distance(uint16_t *lidar_scan,int start,int end);

/*fungsi ini untuk menghirung besar dan arah pergerakan robot untuk menuju pada suatu titik
 * posisi merupakan posisi dalam formasi
 * cartDist merupakan koordinat kartesian dari titik yang dituju
 * vector merupakan besar dan arah dari posisi robot mnuju titik tujuan
 * */
void get_vector_distance(int posisi,int *cartDist,int *vector);

#endif
