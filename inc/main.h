#ifndef MAIN_H_
#define MAIN_H_

#ifdef __cplusplus
extern "C"{
#endif

#include <man_rcc_stm32f4.h>
#include <man_uart_stm32f4.h>
#include <man_adc_stm32f4.h>
#include <man_i2c_stm32f4.h>
#include <man_tim_stm32f4.h>
#include <man_fpu_stm32f4.h>
#include <man_flash_stm32f4.h>
#include <man_exti_stm32f4.h>
#include "math.h"
#include "string.h"
#include "stdlib.h"
#include "stdio.h"
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include <i2c_lcd.h>
#include <MQ_sensor.h>
#include <lidar.h>
#include <esp01_AT.h>
#include <gy26_compass.h>
#include <fuzzy.h>
#include <particle_swarm.h>

#define bSize 			1808						//jumlah data mentah LIDAR dari DMA
#define RLoad 			1.0							//RLoad pada modul sensor gas
#define Ro_Factor		9.83						//Ro factor untuk gas MQ2
#define add0			0x080E0000					//alamat penyimpanan Ro sebelah kanan
#define add1			0x080E0004					//alamat penyimpanan Ro sebelah kiri
#define add2			0x080E0008					//alamat untuk menyimpan ID
#define PI				57.3

/*konstanta yang digunakan untuk penamaan robot,
 * ip address merupakan ip address tujuan (server),
 * own_ip berisi ip client (ip robot) yang didefinisikan sebagai
 * nilai ip1,ip2,ip3 tergantung dari id robot,
 * ip address di set merupakan ip static,
 * port merupakan port pengiriman data melalui udp,
 * limit merupakan jarak maksimum lidar untuk mendeteksi objek,
 * curve merupakan properties dari sensor gas yang berisi sesuai indeks
 * 0 untuk gradient, 1 untuk pembagi dan 2.3 untuk Y offset,
 * */
char buff1[16]= "TIGA";
char buff2[16]= "SERANGKAI";
char ip_address[13]= "192.168.4.1"; 				//ip address server
char own_ip[13];									//buffer untuk menampung IP address
char ip1[13]= "192.168.4.2";						//ip address static
char ip2[13]= "192.168.4.3";
char ip3[13]= "192.168.4.4";
char mask[13]= "255.255.255.0";
const int port= 4210;								//port esp8266
const uint16_t limit= 1500;							//maximum distance lidar in mm
float curve[3]= {-7.33,3.0,2.3};					//kurva untuk MQ2

/* nilai default apabila terjadi error pada robot yang disebabkan oleh nilai
 * tersimpan pada eeprom hilang. ID merupakan nomor robot, sedangkan pos merupakan
 * state formasi. shadow ID merupakan identitas bayangan dari nomor robot.
 * this robot menunjukkan ID dari robot,id1 adalah id robot ketika terjadi hilang pada eeprom
 * warn merupakan penanda apabila mengalami lost data dalam eeprom
 * */
int ID= 1;											//identity default
int pos= 1;											//pos default
int shadowID= 0;									//shadow identity
int this_robot;										//this robot identity
char id1= 'a';										//robot name default
char warn= ' ';										//! peringatan ketika data eeprom hilang

/*global variable
 * buff merupakan buffer untuk string digunakan untuk transmisi UART
 * buff3 merupakan buffer untuk string digunakan untuk tampilan LCD
 * buff4 merupakan buffer untuk string yang dikirim menuju ground station
 * buffTerima merupakan buffer untuk menampung data dari ground station,
 * command state merupakan state tugas yang dikerjakan oleh robot,
 * 0 untuk stop
 * 1 untuk mentrace gas
 * 2 untuk membentuk formasi
 * 3 untuk melakukan tugas kombinasi
 * 4 untuk melakukan tugas kombinasi dan perhitungan metaheuristik
 * */
char buff[33];										//buffer untuk tampilan kirim UART
char buff3[33];										//buffer untuk tampilan LCD
char buff4[256];									//kirim data ke ESP
char buffTerima[33];								//buffer untuk terima data
int idx1= 0;
uint8_t stateTerima= 0;
uint8_t commandState;

uint8_t calib_state=0;								//ini variable untuk state kalibrasi sensor gas
uint8_t calib_state1=0;								//ini variable untuk state untuk mengirim heatmap gas

/*gas variable*/
uint16_t adc[2];									//untuk menyimpan nilai ADC (raw data)
float ro[2]= {1.0,1.0};								//nilai resistansi ketika dalam udara tanpa pengotor
uint32_t roTmp[2];									//nilai ro sementara dalam eeprom
float roPol0[100],roPol1[100];						//polling untuk kalibrasi menentukan nilai Ro
int pol= 0,nPol= 100;								//jumlah pooling untuk kalibrasi sensor
float ppm[2];										//nilai PPM gas 0 kiri, 1 kanan
float rs[2];										//nilai resistansi ketika sensor terkena gas dengan pengotor (sensing resistance)
float ppmK[2],ppmK1[2];								//nilai PPM gas after kalman
float tmpPPM[2];									//variable sampling ppm setiap 1 detik
float dppmK;										//perubahan besar ppm gas
float gasFuzzIn[3];									//index 0 untuk resultan gas concentration,1 untuk beda konsentrasi kiri dan kanan, 2 untuk besar perubahan resultan

/*variabel kalman filter untuk gas*/
const int var[2]= {1600,1600};						//variance untuk kalman sensor gas
float Pc[2]= {0.0,0.0};								//covariance prediction
float G[2]= {0.0,0.0};								//kalman gain
float P[2]= {0.0,0.0};								//covariance estimasi
float Xp[2]= {0.0,0.0};								//nilai x prediksi
float Zp[2]= {0.0,0.0};								//nilai x prediksi
float Xe[2]= {0.0,0.0};								//nilai x estimasi
float varProc= 10.0;								//covariance perubahan

/*fuzzy rule untuk gas*/
int ruleGas[2][63]={{S,S,S,S,S,S,S,
					 S,S,S,S,S,S,S,
					 D,D,D,D,D,D,D,
					 S,S,S,S,S,S,S,
					 M,M,M,M,M,M,M,
					 J,J,J,J,J,J,J,
					 S,S,S,S,S,S,S,
					 S,S,S,S,S,S,S,
					 J,J,J,J,J,J,J},				//sb y
				    {Z,Z,Z,Z,Z,Z,Z,
				     Z,Z,Z,Z,Z,Z,Z,
				     N,N,N,Z,-N,-N,-N,
					 Z,Z,Z,Z,Z,Z,Z,
				     C,N,N,Z,-N,-N,-C,
					 L,C,N,Z,-N,-L,-C,
				     Z,Z,Z,Z,Z,Z,Z,
					 Z,Z,Z,Z,Z,Z,Z,
				     L,C,N,Z,-N,-C,-L}};			//teta

int tmpInference[63];								//inference result
int uppm[3];										//membership function of gas concentration
int udppm[3];										//membership function of gas concentration changes
int uselisih[7];									//selisih ppm kanan dan ppm kiri
int GasParam[2];									//output fuzzy 0 untuk step gerakan, 1 untuk orientasi (arah gerakan)

/*lidar variable*/
uint8_t liBuff[bSize];								//variable untuk menyimpan data mentah LIDAR (terhubung dengan DMA)
uint16_t dist[360],dist1[360],ddist[360];			//variabel penyimpan jarak dan perubahan jarak
int shadow_teta[360],shadow_gama[360];				//variabel untuk menyimpan objek2 terdeteksi
int teta[2],teta1[2],teta2[2];
uint16_t gama[2];									//teta sudut,teta1 sudut(n-1),gama jarak,gama1 jarak(n-1), 0 untuk fnmax, 1 untuk fnmin
int gama1[2],gama2[2];
int xval[2],yval[2],xval1[2],yval1[2],xvalK[2],yvalK[2];			//kalman filter untuk LiDAR
int tetaK[2],gamaK[2];												//hasil kalman filter LiDAR

/* variabel kalman filter untuk lidar*/
int mea[2][4];														//sampling measurement
int Xe1[2][4],Xp1[2][4];											//nilai prediksi dan estimasi
float Pe1[2][4],Pp1[2][4],G1[2][4];									//covariance estimasi dan prediksi
float var1[2][4]= {{50.0,50.0,25.0,25.0},{50.0,50.0,25.0,25.0}};	//error variance of measurement
float varProc1[2][4]= {{4,4,0.5,0.5},{4,4,0.5,0.5}};				//variance of movement

/*fuzzy rule lidar*/
int LidarParam[2];									//output fuzzy lidar 0 untuk step gerakan, 1 untuk orientasi (arah gerakan)
int ruleLidar1[2][81]= {{D,S,D,D,S,D,D,S,D
						,D,S,D,D,S,D,D,S,D
						,D,S,D,D,S,D,D,S,D
						,D,M,M,D,M,M,D,M,M
						,D,M,M,D,M,M,D,M,M
						,D,M,M,D,M,M,D,M,M
						,M,J,J,M,J,J,M,J,J
						,M,J,J,M,J,J,M,J,J
						,M,J,J,M,J,J,M,J,J},
						{-N,-N,-N,-C,-C,-C,-L,-L,-L
						,Z,Z,Z,Z,Z,Z,Z,Z,Z
						,N,N,N,C,C,C,L,L,L
						,-N,-N,-N,-C,-C,-C,-C,-C,-C
						,Z,Z,Z,Z,Z,Z,Z,Z,Z
						,N,N,N,C,C,C,C,C,C
						,-C,-C,-C,-C,-C,-C,-L,-L,-L
						,Z,Z,Z,Z,Z,Z,Z,Z,Z
						,C,C,C,C,C,C,L,L,L}};

int tmpInferenceLidar1[81];							//inference result
int lidarFuzzIn[4];									//0 untuk error jarak, 1 error arah, 2 error d jarak, 3 error d arah
int mag1,pol1;
int uEmag1[3],udEmag1[3],uEpol1[3],udEpol1[3];		//membership function untuk error
int cartDestinationDist[2],vektor[2];
int bentukForm[2]= {1000,30};

/*variabel kompas dan odometery*/
int simpangan;										//orientasi mula-mula robot
int XeOrient,XpOrient;								//estimasi nilai kompas
float PeOrient,PpOrient,GOrient;					//prediksi nilai kompas
float varOrient= 16.0;
float varProcOrient= 0.5;

int refOrient,actOrient,relativeOrient,relativeOrient1,relativeOrient_step;	//orientasi robot
int encoder[2];					  											//0 kiri, 1 kanan
int cartesianG[3];				  											//global cartesian chart 0 untuk x, 1 untuk y 2 untuk orientasi
int step[2],stepSP[2],actStep[2]; 											//0 untuk y dan 1 untuk teta actual, nilai setpoint besar Y dan arah

int dirE[2];										//error
int dirI[2];										//integral error
int dirD[2];										//dif eroor
int dirE1[2];										//prev error

/*variabel motor control*/
int kP[2]= {12,8};									//{12,0};
float kI[2]= {0.02,0.2};							//{0.005,0.0};
float kD[2]= {2.0,5.0};								//{2.0,0.0};
int pid[2];											//pwm value

/*variabel metaheuristic*/
int veloMag[2],veloDir[2];							//kecepatan relative robot lain dan arahnya
int bestOther[2];									//posisi robot lain dengan fitness terbesar 1 jarak 0 sudut
int arahBestOther;
int fn[3];											//menyimpan fitness function 3 robot, 0 myfitness, 1 dan 2 robot lain
float matrice[2]= {0,0};							//matrix untuk penjumlah di behavior-based formation control
uint8_t meta= 0;
uint8_t state= 0;
uint8_t tim5state= 0;

#ifdef __cplusplus
}
#endif

#endif /* MAIN_H_ */
