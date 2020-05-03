/**
  ******************************************************************************
  * @file    main.c
  * @author  Ac6
  * @version V1.0
  * @date    01-December-2013
  * @brief   Default main function.
  ******************************************************************************
*/

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

#define bSize 			1808	//jumlah data mentah LIDAR dari DMA
#define RLoad 			1.0		//RLoad pada modul sensor gas
#define Ro_Factor		9.83	//Ro factor untuk gas MQ2
#define add0			0x080E0000	//alamat penyimpanan Ro sebelah kanan
#define add1			0x080E0004	//alamat penyimpanan Ro sebelah kiri
#define add2			0x080E0008	//alamat untuk menyimpan ID
#define PI				57.3

//constanta
char buff1[16]= "TIGA";
char buff2[16]= "SERANGKAI";
char ip_address[13]= "192.168.4.1"; //ip address server
char own_ip[13];
char ip1[13]= "192.168.4.2";	//ip address static
char ip2[13]= "192.168.4.3";
char ip3[13]= "192.168.4.4";
char mask[13]= "255.255.255.0";
const int port= 4210;	//port
const uint16_t limit= 3000;	//maximum distance lidar in mm

float curve[3]= {-1.7,0.6,4};	//kurva untuk MQ2
const uint8_t LCD= PCF8574;	//alamat device LCD
const uint8_t LCD1= PCF8574A;	//alamat device LCD

//identity
int ID= 1;			//identity default
int pos= 1;			//pos default
int shadowID= 0;	//shadow identity
int this_robot;		//this robot identity
char id1= 'a';		//robot name default
char warn= ' ';

//global variable
char buff[33];			//buffer untuk tampilan kirim UART
char buff3[33];			//buffer untuk tampilan LCD
char buff4[1024];			//buffer untuk kirim ke esp
int calib_state=0;		//ini variable untuk state kalibrasi sensor gas
int calib_state1=0;		//ini variable untuk state kalibrasi kompas
int menu_state=0;
int cntr=0;

//gas variable
uint16_t adc[2];		//untuk menyimpan nilai ADC (raw data)
float ro[2]= {1.0,1.0};	//nilai resistansi ketika dalam udara tanpa pengotor
uint32_t roTmp[2];
float roPol0[100];	//polling untuk kalibrasi menentukan nilai Ro
float roPol1[100];	//polling untuk kalibrasi menentukan nilai Ro
float ppm[2];		//nilai PPM gas
float ppmK[2];		//nilai PPM gas after kalman
float ppmK1[2];
float dppmK[2];
int pol= 0;			//indeks untuk menyimpan Ro dalam kalibrasi
float rs[2];		//nilai resistansi ketika sensor terkena gas dengan pengotor (sensing resistance)

//kalman filter untuk gas
const int var[2]= {1600,1600};	//variance untuk kalman sensor gas
float Pc[2]= {0.0,0.0};
float G[2]= {0.0,0.0};
float P[2]= {0.0,0.0};
float Xp[2]= {0.0,0.0};
float Zp[2]= {0.0,0.0};
float Xe[2]= {0.0,0.0};
float varProc= 10.0;

//fuzzy rule untuk gas
int ruleGas[2][81]={{S,D,D,D,S,D,D,D,M,
					 D,D,D,D,D,D,D,D,M,
					 D,D,D,D,M,M,D,M,M,
					 D,D,D,D,D,D,D,D,M,
					 D,D,M,D,M,M,M,M,J,
					 M,M,M,M,J,J,M,J,J,
					 D,D,D,D,D,D,D,D,M,
					 M,M,M,M,J,J,M,J,J,
					 M,M,M,M,J,J,M,J,J},		//sb y
				    {Z,N,C,-N,Z,N,-C,-N,Z,
				     N,N,C,Z,N,N,Z,Z,N,
				     N,N,L,Z,N,C,Z,Z,N,
				     -N,Z,Z,-N,-N,Z,-C,-N,-N,
				     Z,N,C,-N,Z,N,-C,-N,Z,
				     Z,N,L,Z,N,N,Z,Z,Z,
				     -N,Z,Z,-N,-N,Z,-L,-C,-N,
				     Z,Z,Z,-N,-N,Z,-L,-N,Z,
				     Z,N,N,-N,Z,Z,-N,Z,Z}};	//teta
int tmpInference[81];	//inference result
int uppm[2][3];			//membership function of gas concentration
int udppm[2][3];		//membership function of gas concentration changes
int GasParam[2];		//output fuzzy

//lidar variable
uint8_t liBuff[bSize];	//variable untuk menyimpan data mentah LIDAR (terhubung dengan DMA)
uint16_t dist[360],dist1[360],shadow_gama[360];
int shadow_teta[360];
int ddist[360];	//variable jarak dalam mm yang telah diolah
int teta[2],teta1[2];
uint16_t gama[2];	//teta sudut,teta1 sudut(n-1),gama jarak,gama1 jarak(n-1), 0 untuk fnmax, 1 untuk fnmin
int gama1[2];
int xval[2],yval[2],xval1[2],yval1[2],xvalK[2],yvalK[2];
int tetaK[2],gamaK[2];

//kalman filter untuk lidar
int mea[2][4];	//sampling measurement
int Xe1[2][4],Xp1[2][4];
float Pe1[2][4],Pp1[2][4],G1[2][4];
float var1[2][4]= {{16.0,16.0,0.01,0.01},{16.0,16.0,0.01,0.01}};
float varProc1= 0.04;

//fuzzy untuk formation control
int ruleLidar[3][2][9]={{{J,M,J,S,S,D,S,S,S},
						 {Z,Z,Z,Z,Z,Z,Z,Z,Z}},
					    {{M,S,D,D,S,D,J,M,J},
					     {-L,Z,-C,Z,Z,N,Z,Z,N}},
					    {{M,S,D,D,S,D,J,M,J},
						 {L,Z,C,Z,Z,-N,Z,Z,-N}}};
int tmpInferenceLidar[9];	//inference result of fuzzy
int sudut_apit,resultan_dist;	//input fuzzy
int ualpha[3];	//membership function of sudut apit
int uresultan[3];	//membership function of resultan
int LidarParam[2];	//output fuzzy lidar
int lidarSP[3]= {60,600,1035};	//60 derajat dan 600mm
int lidarE[2];

//kompas dan odometery
int orientasi,orientasi1,dorientasi;	//orientasi robot
int avgFil[5];	//average filter
int encoder[2];	//0 kiri, 1 kanan
int cartesianG[3];	//global cartesian chart
int cartesian[3];	//cartesian step
int dirS[2]; //0 untuk y dan 1 untuk teta actual, nilai setpoint besar Y dan arah
int dir_sh[2];	//direction and orientation actual perstep
int dirE[2];
int dirI[2];
int dirD[2];
int dirE1[2];

//motor control
int kP[2]= {4,6};
float kI[2]= {0.1,0.001};
float kD[2]= {0.3,0.5};
int pid[2];

//metaheuristic
int veloMag[2],dir[2];
int fn[3];
int tim4state= 0;
float matrice[2]= {0,0};	//matrix untuk penjumlah di behavior-based formation control
int meta= 0;
int state= 0;

//perebutan posisi 2 dan 3
int upos[2][4];
int tmpInferecePos[16];
int rule_pos[16]= {2,2,3,3,2,2,3,3,3,2,3,3,2,2,3,3};	//rule posisi dalam formasi

int main(void){
	//inisialisasi peripheral////////////////////////////////////////////////////////////////////
	SystemClock_Config();
	GPIO_Init();
	ADC_Init();
	I2C1_Init();
	UART_Init();
	FPU_Init();
	TIM2_Init();
	TIM1_Init();
	EXTI_Init();
	HAL_Delay(100);

	//wellcome screen////////////////////////////////////////////////////////////////////////////
	i2c_lcdInit(I2C1,LCD);
	i2c_lcdClear(I2C1,LCD);
	i2c_lcdSetCursor(I2C1,LCD,0,0);
	i2c_lcdWriteStr(I2C1,LCD,buff1);
	i2c_lcdSetCursor(I2C1,LCD,1,0);
	i2c_lcdWriteStr(I2C1,LCD,buff2);

	//restart ESP////////////////////////////////////////////////////////////////////////////////
	HAL_Delay(500);
	esp_restart(USART2);
	HAL_Delay(500);

	//load ID///////////////////////////////////////////////////////////////////////////////////
	this_robot= *(uint32_t*) add2;
	if(this_robot==1){
		id1= 'a';
		ID= 1;
		warn= ' ';
		for(int i=0;i<13;i++){
			own_ip[i]=ip1[i];
		}
	}
	else if(this_robot==2){
		id1= 'b';
		ID= 2;
		warn= ' ';
		for(int i=0;i<13;i++){
			own_ip[i]=ip2[i];
		}
	}
	else if(this_robot==3){
		id1= 'c';
		ID= 3;
		warn= ' ';
		for(int i=0;i<13;i++){
			own_ip[i]=ip3[i];
		}
	}
	else{
		id1= 'c';
		ID= 3;
		warn= '!';
		for(int i=0;i<13;i++){
			own_ip[i]=ip3[i];
		}
	}

	//stop lidar////////////////////////////////////////////////////////////////////////////////
	lidar_command(USART3,RP_STOP);

	//connect esp to wifi///////////////////////////////////////////////////////////////////////
	i2c_lcdClear(I2C1,LCD);
	i2c_lcdSetCursor(I2C1,LCD,0,0);
	i2c_lcdWriteStr(I2C1,LCD,"Connecting to:");
	i2c_lcdSetCursor(I2C1,LCD,1,0);
	i2c_lcdWriteStr(I2C1,LCD,"swarm");
	connect_ssid(USART2,"swarm","bismillah");
	static_ip(USART2,own_ip,ip_address,mask);
	set_udp_connection(USART2,ip_address,ID,port);

	//wifi connected////////////////////////////////////////////////////////////////////////////
	i2c_lcdClear(I2C1,LCD);
	i2c_lcdSetCursor(I2C1,LCD,0,0);
	i2c_lcdWriteStr(I2C1,LCD,"Connected..!!");
	i2c_lcdSetCursor(I2C1,LCD,1,0);
	i2c_lcdWriteStr(I2C1,LCD,own_ip);
	HAL_Delay(1000);

	//load nilai Ro yang sudah tersimpan dari nonvolatile memory////////////////////////////////
	roTmp[0]= *(uint32_t*) add0;
	roTmp[1]= *(uint32_t*) add1;
	ro[0]= (float)roTmp[0]/100;
	ro[1]= (float)roTmp[1]/100;

	//reset compass dan pooling untuk nilai awalan/////////////////////////////////////////////
	for(int i=0;i<5;i++){
		reset_gy26(I2C1);
		HAL_Delay(1);
	}
	for(int i=0;i<10;i++){
		orientasi= (get_angle(I2C1)%360);	//sudut dalam derajat
		HAL_Delay(10);
	}
	HAL_Delay(1000);

	//initial peripheral tambahan1/////////////////////////////////////////////////////////////
	UART2_InterruptReception();
	DMA1_Stream1_Init((uint32_t)&liBuff,(uint32_t)&USART3->DR,bSize);
	UART_DMA_reception(USART3);
	TIM5_Init();
	TIM3_Init();
	TIM6_Init();

	while(1){
		//untuk program diluar interrupt subroutine/////////////////////////////////
		if(calib_state==1){//process during MQ sensor calibration
			i2c_lcdClear(I2C1,LCD);
			i2c_lcdSetCursor(I2C1,LCD,0,2);
			i2c_lcdWriteStr(I2C1,LCD,"calibrating");
			i2c_lcdSetCursor(I2C1,LCD,1,3);
			i2c_lcdWriteStr(I2C1,LCD,"MQ sensor");
		}
		else if(calib_state==2){//MQ calibration done
			i2c_lcdClear(I2C1,LCD);
			i2c_lcdSetCursor(I2C1,LCD,0,4);
			i2c_lcdWriteStr(I2C1,LCD,"done..!!");
			HAL_Delay(1000);
			calib_state= 0;
			ro[0]= MQ_Calibration(roPol0,Ro_Factor,pol);
			ro[1]= MQ_Calibration(roPol1,Ro_Factor,pol);
			i2c_lcdClear(I2C1,LCD);
			i2c_lcdSetCursor(I2C1,LCD,0,0);
			sprintf(buff3,"Ro l= %.2fk",ro[0]);
			i2c_lcdWriteStr(I2C1,LCD,buff3);
			i2c_lcdSetCursor(I2C1,LCD,1,0);
			sprintf(buff3,"Ro r= %.2fk",ro[1]);
			i2c_lcdWriteStr(I2C1,LCD,buff3);
			//simpan di eeprom atau flash memory (non volatile memory)
			HAL_FLASH_Unlock();
			FLASH_Erase_Sector(FLASH_SECTOR_11,VOLTAGE_RANGE_3);
			HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,add0,(uint32_t)(ro[0]*100));
			HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,add1,(uint32_t)(ro[1]*100));
			HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,add2,(uint32_t)(this_robot));
			HAL_FLASH_Lock();
			pol= 0;
		}
		else if(calib_state1==1){//compass calibrating
			i2c_lcdClear(I2C1,LCD);
			i2c_lcdSetCursor(I2C1,LCD,0,0);
			i2c_lcdWriteStr(I2C1,LCD,"compass");
			i2c_lcdSetCursor(I2C1,LCD,1,0);
			sprintf(buff3,"%d,%d,%d",orientasi,orientasi1,dorientasi);
			i2c_lcdWriteStr(I2C1,LCD,buff3);
		}
		else if(calib_state1==2){//compass calibrating done
			i2c_lcdClear(I2C1,LCD);
			i2c_lcdSetCursor(I2C1,LCD,0,4);
			i2c_lcdWriteStr(I2C1,LCD,"done..!!");
			HAL_Delay(1000);
			calib_state1=0;
		}
		else if(menu_state==1){//menu for localization
			i2c_lcdClear(I2C1,LCD);
			i2c_lcdSetCursor(I2C1,LCD,0,2);
			i2c_lcdWriteStr(I2C1,LCD,"locate others");
			i2c_lcdSetCursor(I2C1,LCD,1,5);
			i2c_lcdWriteStr(I2C1,LCD,"Robots");
			menu_state++;
			HAL_Delay(100);
		}
		else if(menu_state==2){	//menampilkan posisi kawan robot
			i2c_lcdClear(I2C1,LCD);
			i2c_lcdSetCursor(I2C1,LCD,0,0);
			sprintf(buff3,"A= %4dmm|%3d",gama[0],teta[0]);
			i2c_lcdWriteStr(I2C1,LCD,buff3);
			i2c_lcdSetCursor(I2C1,LCD,1,0);
			sprintf(buff3,"B= %4dmm|%3d",gama[1],teta[1]);
			i2c_lcdWriteStr(I2C1,LCD,buff3);
		}
		else if(menu_state==3){//menu for measuring velocity
			i2c_lcdClear(I2C1,LCD);
			i2c_lcdSetCursor(I2C1,LCD,0,0);
			i2c_lcdWriteStr(I2C1,LCD,"measure velocity");
			i2c_lcdSetCursor(I2C1,LCD,1,5);
			i2c_lcdWriteStr(I2C1,LCD,"Robots");
			menu_state++;
			HAL_Delay(100);
		}
		else if(menu_state==4){	//menampilkan kecepatan relatif kawan robot
			i2c_lcdClear(I2C1,LCD);
			i2c_lcdSetCursor(I2C1,LCD,0,0);
			sprintf(buff3,"A=%d mm/s |%d",abs(veloMag[0]),dir[0]);
			i2c_lcdWriteStr(I2C1,LCD,buff3);
			i2c_lcdSetCursor(I2C1,LCD,1,0);
			sprintf(buff3,"B=%d mm/s |%d",abs(veloMag[1]),dir[1]);
			i2c_lcdWriteStr(I2C1,LCD,buff3);
		}
		else{//default display
			i2c_lcdClear(I2C1,LCD);
			sprintf(buff3,"%.1f",ppmK[0]);
			i2c_lcdSetCursor(I2C1,LCD,0,0);
			i2c_lcdWriteStr(I2C1,LCD,buff3);
			i2c_lcdSetCursor(I2C1,LCD,0,6);
			sprintf(buff3,"|%c%c|",warn,id1);
			i2c_lcdWriteStr(I2C1,LCD,buff3);
			sprintf(buff3,"%.1f",ppmK[1]);
			i2c_lcdSetCursor(I2C1,LCD,0,10);
			i2c_lcdWriteStr(I2C1,LCD,buff3);
			sprintf(buff3,"%.2fk",ro[0]);
			i2c_lcdSetCursor(I2C1,LCD,1,0);
			i2c_lcdWriteStr(I2C1,LCD,buff3);
			i2c_lcdSetCursor(I2C1,LCD,1,6);
			sprintf(buff3,"| %d|",pos);
			i2c_lcdWriteStr(I2C1,LCD,buff3);
			sprintf(buff3,"%.2fk",ro[1]);
			i2c_lcdSetCursor(I2C1,LCD,1,10);
			i2c_lcdWriteStr(I2C1,LCD,buff3);
		}

		//disini yang harus dihapus ketika metaheuristik dipakai/////////////////////
		if(!meta){
			pos= this_robot;
		}

		//get compass val/////////////////////////////////////////////////////////////
		for(int i=0;i<3;i++){
			orientasi1= get_angle(I2C1)%360;	//sudut dalam derajat
			HAL_Delay(1);
		}
		int dorientasi1= orientasi1-orientasi;
		if(dorientasi1>= 180){
			dorientasi1= -1*(((360-orientasi1)+orientasi)%360);
		}
		else if(dorientasi1<= -180){
			dorientasi1= ((360-orientasi)+orientasi1)%360;
		}

		avgFil[0]=dorientasi1;
		int mean=0;
		for(int i=0;i<5;i++){
			mean= mean+avgFil[i];
		}
		dorientasi= mean/5;
		for(int i=4;i>0;i--){
			avgFil[i]=avgFil[i-1];
		}

		HAL_Delay(200);
	}
}

//timer 4 interrupt subroutine setial 0.2 detik sekali untuk sampling gas
void TIM4_IRQHandler(void){
	if(TIM4->SR&(1UL)){	//update generation due to overflow counter
		GPIOD->ODR^= (1UL<<12U);

		//odometry here//////////////////////////////////////////////////////////////////
		if(pid[1]>=0){					//get encoder value
			encoder[1]= 6*(TIM2->CNT);	//kecepatan dalam mm/s
		}
		else if(pid[1]<0){
			encoder[1]= -6*(TIM2->CNT);
		}
		if(pid[0]>=0){
			encoder[0]= 6*(TIM1->CNT);
		}
		else if(pid[0]<0){
			encoder[0]= -6*(TIM1->CNT);
		}

		//resultant velocity
		int magV= (int)(encoder[0]+encoder[1])/2;
		//cartesian step
		cartesian[0]=cartesian[0]+(int)(magV*sin(dorientasi/PI));
		cartesian[1]=cartesian[1]+(int)(magV*cos(dorientasi/PI));
		cartesian[2]=dorientasi;
		//global cartesian robot position
		cartesianG[0]= (cartesianG[0]+cartesian[0]);
		cartesianG[1]= (cartesianG[1]+cartesian[1]);
		cartesianG[2]= cartesian[2];
		//robot direction step
		dir_sh[0]= dir_sh[0]+magV;
		dir_sh[1]= cartesian[2];

		//sampling gas sementara single conversion,mencoba untuk simultan////////////////////////////
		adc[0]= ADC_getVal(ADC1);
		adc[1]= ADC_getVal(ADC2);
		rs[0]= MQ_Get_Resistance(adc[0],RLoad);
		rs[1]= MQ_Get_Resistance(adc[1],RLoad);
		ppm[0]= MQ_Get_PPM((float)rs[0]/ro[0],curve);
		ppm[1]= MQ_Get_PPM((float)rs[1]/ro[1],curve);

		//kalman filter dari sensor gas//////////////////////////////////////////////////////////////
		for(int i=0;i<2;i++){
			Pc[i]= P[i]+varProc;
			G[i]= (float)Pc[i]/(Pc[i]+var[i]);
			P[i]= (1-G[i])*Pc[i];
			Xp[i]= Xe[i];
			Zp[i]= Xp[i];
			Xe[i]= (G[i]*(ppm[i]-Zp[i]))+Xp[i];
			ppmK[i]= Xe[i];
			dppmK[i]= ppmK[i]-ppmK1[i];
		}

		//pooling during calibration
		if(calib_state==1){
			roPol0[pol]=rs[0];
			roPol1[pol]=rs[1];
			pol++;
			if(pol>=100)calib_state++;
		}

		//sampling data posisi robot lidar////////////////////////////////////////////////////////////
		convert_cartesian(gama,teta,yval,xval);
		for(int i=0;i<2;i++){
			mea[i][0]=yval[i];
			mea[i][1]=xval[i];
			mea[i][2]=yval[i]-yval1[i];
			mea[i][3]=xval[i]-xval1[i];
		}

		//kalman filter untuk posisi robot dalam kartesian////////////////////////////////////////////
		if(!state){
			for(int i=0;i<2;i++){
				for(int j=0;j<4;j++){
					if(j==0){
						Xe1[i][j]= yval[i];
					}
					else if(j==1){
						Xe1[i][j]= xval[i];
					}
					else{
						Xe1[i][j]= 0;
					}
				}
			}
			state= 1;
		}

		//predict
		for(int i=0;i<2;i++){
			for(int j=0;j<4;j++){
				if(j<2){
					Xp1[i][j]= Xe1[i][j]+(Xe1[i][j+2]/5);
				}
				else{
					Xp1[i][j]= Xe1[i][j];
				}
				Pp1[i][j]= Pe1[i][j]+varProc1;
			}
		}
		//estimate
		for(int i=0;i<2;i++){
			for(int j=0;j<4;j++){
				G1[i][j]= (float)Pp1[i][j]/(Pp1[i][j]+var1[i][j]);
				Xe1[i][j]= Xp1[i][j]+(G1[i][j]*(mea[i][j]-Xp1[i][j]));
				Pe1[i][i]= (1-G1[i][j])*Pp1[i][j];
			}
		}

		for(int i=0;i<2;i++){
			yvalK[i]=Xe1[i][0];
			xvalK[i]=Xe1[i][1];
			if((yvalK[i]<0)&(xvalK[i]<0)){
				tetaK[i]= (int)(PI*atan((float)xvalK[i]/(yvalK[i]+1)))+180;
			}
			else if((yvalK[i]<0)&(xvalK[i]>=0)){
				tetaK[i]= 180+(int)(PI*atan((float)xvalK[i]/(yvalK[i]+1)));
			}
			else if((yvalK[i]>=0)&(xvalK[i]<0)){
				tetaK[i]= 360+(int)(PI*atan((float)xvalK[i]/(yvalK[i]+1)));
			}
			else{
				tetaK[i]= (int)(PI*atan((float)xvalK[i]/(yvalK[i]+1)));
			}
			gamaK[i]= sqrt(pow(xvalK[i],2)+pow(yvalK[i],2));
		}

		//disini mulai fuzzynya////////////////////////////////////////////////////////////
		sudut_apit= (360+abs(tetaK[0]-tetaK[1]))%360;	//sudut yang dibentuk oleh 2 robot yang terdeteksi
		if(sudut_apit>=180)sudut_apit= abs(((tetaK[1]+180)%360-180)-((tetaK[0]+180)%360-180));
		resultan_dist= sqrt(pow(gamaK[0],2)+pow(gamaK[1],2)+(2*gamaK[0]*gamaK[1]*cos(sudut_apit/PI)));
		//hitung error lidar
		lidarE[0]= sudut_apit-lidarSP[0];
		lidarE[1]= resultan_dist-lidarSP[2];

		if((dir_sh[0]>= dirS[0])){
			GPIOD->ODR^= (1UL<<15U);
			//fuzzy yang baru untuk gerak menuju gas///////////////////////////////////////
			uppm[0][0]= fs_trapesium_sikukiri(ppmK[0],0,2.5,6.5,100);
			uppm[0][1]= fs_segitiga(ppmK[0],3.4,40.5,70,100);
			uppm[0][2]= fs_trapesium_sikukanan(ppmK[0],35.4,100.5,20000,100);
			uppm[1][0]= fs_trapesium_sikukiri(ppmK[1],0,2.5,6.5,100);
			uppm[1][1]= fs_segitiga(ppmK[1],3.4,40.5,70,100);
			uppm[1][2]= fs_trapesium_sikukanan(ppmK[1],35.4,100.5,20000,100);

			udppm[0][0]= fs_trapesium_sikukiri(dppmK[0],-20000,-38.5,0,100);
			udppm[0][1]= fs_segitiga(dppmK[0],-55.5,0,55.5,100);
			udppm[0][2]= fs_trapesium_sikukanan(dppmK[0],0,37,20000,100);
			udppm[1][0]= fs_trapesium_sikukiri(dppmK[1],-20000,-38.5,0,100);
			udppm[1][1]= fs_segitiga(dppmK[1],-55.5,0,55.5,100);
			udppm[1][2]= fs_trapesium_sikukanan(dppmK[1],0,37,20000,100);

			inference(uppm,udppm,tmpInference);	//tmpInference should be &
			center_area(tmpInference,81,ruleGas,GasParam);	//gas param should be &

			//disini fuzzy untuk lidar//////////////////////////////////////////////////////
			ualpha[0]=fs_trapesium_sikukiri(lidarE[0],-360,-25,0,100);						//ualpha[0]=fs_trapesium_sikukiri(sudut_apit,0,25,45,100);
			ualpha[1]= fs_segitiga(lidarE[0],-15,0,15,100);								//ualpha[1]= fs_segitiga(sudut_apit,30,60,90,100);
			ualpha[2]= fs_trapesium_sikukanan(lidarE[0],0,25,360,100);				//ualpha[2]= fs_trapesium_sikukanan(sudut_apit,75,105,360,100);

			uresultan[0]= fs_trapesium_sikukiri(lidarE[1],-3000,-350,-180,100);			//uresultan[0]= fs_trapesium_sikukiri(resultan_dist,0,450,900,100);
			uresultan[1]= fs_segitiga(lidarE[1],-250,0,250,100);					//uresultan[1]= fs_segitiga(resultan_dist,600,1040,1400,100);
			uresultan[2]= fs_trapesium_sikukanan(lidarE[1],180,350,3000,100);	//uresultan[2]= fs_trapesium_sikukanan(resultan_dist,1200,1350,10000,100);

			inference_lidar(uresultan,ualpha,tmpInferenceLidar);	//tmpInference should be &
			center_area_lidar((pos-1),tmpInferenceLidar,9,ruleLidar,LidarParam);	//lidar param should be &

			//setpoint declaration//////////////////////////////////////////////////////////
			dirS[0]= (matrice[0]*GasParam[0])+(matrice[1]*LidarParam[0]);
			dirS[1]= dirS[1]+(matrice[0]*GasParam[1])+(matrice[1]*LidarParam[1]);

			//limitation////////////////////////////////////////////////////////////////////
			if(dirS[1]>= 60){
				dirS[1]= 60;
			}
			else if(dirS[1]<= -60){
				dirS[1]= -60;
			}

			//step reset after fuzzy process///////////////////////////////////////////////
			cartesian[0]=0;
			cartesian[1]=0;
			dir_sh[0]= 0;
		}

		//perebutan formasi////////////////////////////////////////////////////////////////
		//menghitung kecepatan para robot
		measure_velocity(gamaK,gama1,dorientasi,tetaK,teta1,veloMag,dir,tim4state);	//mencari kecepatan tiap2 robot
		fn[0]= (int)magV;
		fn[1]= (int)(((gamaK[0]/10)*cos(tetaK[0]/PI))+veloMag[0]);
		fn[2]= (int)(((gamaK[1]/10)*cos(tetaK[1]/PI))+veloMag[1]);
		if(meta){
			if((fn[0]>fn[1])&&(fn[0>fn[2]])){
				pos= 1;
			}
			else{
				//fuzzy for position
				upos[0][0]= fs_kotak(tetaK[0],0,90,100);
				upos[0][1]= fs_kotak(tetaK[0],90,180,100);
				upos[0][2]= fs_kotak(tetaK[0],180,270,100);
				upos[0][3]= fs_kotak(tetaK[0],270,360,100);

				upos[1][0]= fs_kotak(tetaK[1],0,90,100);
				upos[1][1]= fs_kotak(tetaK[1],90,180,100);
				upos[1][2]= fs_kotak(tetaK[1],180,270,100);
				upos[1][3]= fs_kotak(tetaK[1],270,360,100);

				inference_formation(upos,tmpInferecePos);
				pos= center_area_formation(tmpInferecePos,16,rule_pos);
			}
		}

		//kirim ke ESP setiap 0.2 detik/////////////////////////////////////////////////////
		sprintf(buff4,"%c;%6d;%6d;%4d;%3d;%3d;%3d;%3d;%4d;%4d;%9d;%9d;%3d;%4d;%3d;%4d;%4d;%4d;%4d;%3d",id1,(int)(ppmK[0]*10),(int)(ppmK[1]*10),dorientasi,dirS[0],dirS[1],cartesian[0],cartesian[1],pid[0],pid[1],cartesianG[0],cartesianG[1],tetaK[0],gamaK[0],tetaK[1],gamaK[1],fn[0],fn[1],fn[2],sudut_apit);
		send_udp(USART2,ID,buff4);

		//akhir dari satu subroutine dengan menshift nilai//////////////////////////////////
		for(int i=0;i<2;i++){
			yval1[i]= yvalK[i];
			xval1[i]= xvalK[i];
			teta1[i]= tetaK[i];
			gama1[i]= gamaK[i];
			ppmK1[i]= ppmK[i];
		}

		TIM2->EGR|= (1UL);
		TIM1->EGR|= (1UL);
		magV= 0;

		TIM4->SR&= ~(1UL);
	}
}

void DMA1_Stream1_IRQHandler(void){
	if(DMA1->LISR &(1UL<<11U)){
		//stop lidar//////////////////////////////////////////////////////////////////////////
		lidar_command(USART3,RP_STOP);

		GPIOD->BSRR|= (1UL<<14U);
		//lidar conversion to polar coordinat/////////////////////////////////////////////////
		rplidar_conversion(liBuff,bSize,limit,dist,tim4state);
		probabilistic_filter(dist,dist1,1,8,5);	//probabilistic filter to eliminate outlier

		//find robot location////////////////////////////////////////////////////////////////
		int stdev,mean;
		derivate_graph(dist1,ddist,360,&stdev,&mean);	//turunan graph
		find_robot(ddist,dist1,stdev,mean,360,1,shadow_teta,shadow_gama);	//shadow teta and shadow gama should be &
		find_theBest(shadow_gama,shadow_teta,360,tetaK,gamaK,teta,gama,limit,tim4state);	//shadow teta1 and shadow gama1 should be &

		//menghapus temporary variable///////////////////////////////////////////////////////////////
		memset(liBuff,0,sizeof(liBuff));
		memset(shadow_gama,0,sizeof(shadow_gama));
		memset(shadow_teta,0,sizeof(shadow_teta));
		GPIOD->BSRR|= (1UL<<30U);

		if(!tim4state){
			TIM4_Init();
			tim4state= 1;
		}

		DMA1->LIFCR|= (1UL<<11U);
	}
}

void TIM5_IRQHandler(void){		//timer 5 interrupt subroutine setiap 1 detik sekali untuk lidar
	if(TIM5->SR&(1UL)){ //update generation due to overflow counter
		GPIOD->ODR^= (1UL<<13U);
		//lidar start scanning//////////////////////////////////////////////////////////////
		lidar_command(USART3,RP_START);

		TIM5->SR&= ~(1UL);
	}
}

void EXTI0_IRQHandler(void){	//subroutine interrupt untuk kalibrasi sensor MQ
	if(EXTI->PR&(1UL)){
		calib_state= calib_state+1;
		menu_state=0;
		calib_state1=0;
		EXTI->PR|= (1UL);
	}
}

void EXTI1_IRQHandler(void){	//untuk kalibrasi kompas
	if(EXTI->PR&(2UL)){
		TIM1->CNT=0;
		calib_state1=calib_state1+1;
		menu_state=0;
		calib_state=0;
		EXTI->PR|= (1UL<<1U);
	}
}

void EXTI2_IRQHandler(void){	//subroutine untuk melihat posisi robot lain dan melihat kecepatan robot lain
	if(EXTI->PR&(4UL)){
		menu_state= (menu_state+1)%5;
		calib_state=0;
		calib_state1=0;
		EXTI->PR|= (1UL<<2U);
	}
}

void EXTI3_IRQHandler(void){	//belum terpakai
	if(EXTI->PR&(8UL)){
		shadowID= (shadowID+1)%3;
		this_robot= shadowID+1;
		switch(this_robot){
		case 1: id1= 'a';
			break;
		case 2: id1= 'b';
			break;
		case 3: id1= 'c';
			break;
		}
		//simpan ID yang baru///////////////////////////////////////////////////////////////
		HAL_FLASH_Unlock();
		FLASH_Erase_Sector(FLASH_SECTOR_11,VOLTAGE_RANGE_3);
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,add0,(uint32_t)(ro[0]*100));
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,add1,(uint32_t)(ro[1]*100));
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,add2,(uint32_t)(this_robot));
		HAL_FLASH_Lock();

		EXTI->PR|= (1UL<<3U);
	}
}

void TIM6_DAC_IRQHandler(void){		//timer 6 untuk PID kontrol kecepatan roda kanan dan kiri
	if(TIM6->SR&(1UL)){

		//find error////////////////////////////////////////////////////////////////////////
		for(int i=0;i<2;i++){
			dirE[i]= dirS[i]-dir_sh[i];
		}
		//find integrate error and derivative error////////////////////////////////////////
		for(int i=0;i<2;i++){
			dirI[i]= dirI[i]+dirE[i];
			dirD[i]= dirE1[i]-dirE[i];
		}
		//0 kiri, 1 kanan untuk PID////////////////////////////////////////////////////////
		int yaxis[2];
		yaxis[0]= (kP[0]*dirE[0])+(kI[0]*dirI[0])+(kD[0]*dirD[0]);
		yaxis[1]= (kP[0]*dirE[0])+(kI[0]*dirI[0])+(kD[0]*dirD[0]);

		for(int i=0;i<2;i++){
			if(yaxis[i]>=1000)yaxis[i]=1000;
			else if(yaxis[i]<= 0)yaxis[i]=0;
		}

		pid[0]= yaxis[0]+((kP[1]*dirE[1])+(kI[1]*dirI[1])+(kD[1]*dirD[1]));
		pid[1]= yaxis[1]+(-1*((kP[1]*dirE[1])+(kI[1]*dirI[1])+(kD[1]*dirD[1])));

		for(int i=0;i<2;i++){
			if(pid[i]>=1000)pid[i]=1000;
			else if(pid[i]<= -1000)pid[i]=-1000;
		}

		for(int i=0;i<2;i++){
			dirE1[i]= dirE[i];
		}

		//motor jalan///////////////////////////////////////////////////////////////////////
		motor_jalan(TIM3,KIRI,pid[0]);
		motor_jalan(TIM3,KANAN,pid[1]);

		TIM6->SR&= ~(1UL);
	}
}

void USART2_IRQHandler(void){
	//GPIOD->ODR^= (1UL<<15U);
	if(USART2->SR & (1UL<<5U)){
		char uart_temp= USART2->DR;

		if(uart_temp== ']'){	//combination move and metaheuristic
			TIM3->CR1|= (1UL);	//counter enable
			matrice[0]= 1;
			matrice[1]= 1;
			meta=1;
			TIM6->CR1|= (1UL);
		}
		else if(uart_temp== '{'){	//combination move
			TIM3->CR1|= (1UL);	//counter enable
			matrice[0]= 1;
			matrice[1]= 1;
			meta=0;
			TIM6->CR1|= (1UL);
		}
		else if(uart_temp== '('){//gas tracing move
			TIM3->CR1|= (1UL);	//counter enable
			matrice[0]= 1;
			matrice[1]= 0;
			meta=0;
			TIM6->CR1|= (1UL);
		}
		else if(uart_temp== ')'){//lidar move formation
			TIM3->CR1|= (1UL);	//counter enable
			matrice[0]= 0;
			matrice[1]= 1;
			meta=0;
			TIM6->CR1|= (1UL);
		}
		else if(uart_temp== '}'){//stop
			TIM3->CR1&= ~(1UL);	//counter enable
			matrice[0]= 0;
			matrice[1]= 0;
			TIM6->CR1&= ~(1UL);
			pid[0]= 0;
			pid[1]= 0;
			meta=0;
		}
		USART2->SR&= ~(1UL<<5U);
	}
}
