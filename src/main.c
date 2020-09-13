/**
  ******************************************************************************
  * @file    main.c
  * @author  Ac6
  * @version V1.0
  * @date    01-December-2013
  * @brief   Default main function.
  ******************************************************************************
*/
#include "main.h"

int main(void){
	/*inisialisasi peripheral*/
	SystemClock_Config();
	GPIO_Init();
	ADC_Init();
	I2C1_Init();
	UART_Init();
	FPU_Init();
	TIM2_Init();
	TIM1_Init();
	EXTI_Init();
	HAL_Delay(500);

	/*inisialisasi LCD dan set wellcome screen*/
	i2c_lcdInit(I2C1,PCF8574);
	i2c_lcdClear(I2C1,PCF8574);
	i2c_lcdSetCursor(I2C1,PCF8574,0,0);
	i2c_lcdWriteStr(I2C1,PCF8574,buff1);
	i2c_lcdSetCursor(I2C1,PCF8574,1,0);
	i2c_lcdWriteStr(I2C1,PCF8574,buff2);

	/*reset ESP*/
	HAL_Delay(100);
	esp_restart(USART2);
	HAL_Delay(100);
	disable_echo(USART2);

	/*load ID robot*/
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

	/*stop LiDAR*/
	lidar_command(USART3,RP_STOP);

	/*menghubungkan robot ke server*/
	i2c_lcdClear(I2C1,PCF8574);
	i2c_lcdSetCursor(I2C1,PCF8574,0,0);
	i2c_lcdWriteStr(I2C1,PCF8574,"Connecting to:");
	i2c_lcdSetCursor(I2C1,PCF8574,1,0);
	i2c_lcdWriteStr(I2C1,PCF8574,"swarm");
	connect_ssid(USART2,"swarm","bismillah");
	static_ip(USART2,own_ip,ip_address,mask);
	set_udp_connection(USART2,ip_address,ID,port);
	i2c_lcdClear(I2C1,PCF8574);
	i2c_lcdSetCursor(I2C1,PCF8574,0,0);
	i2c_lcdWriteStr(I2C1,PCF8574,"Connected..!!");
	i2c_lcdSetCursor(I2C1,PCF8574,1,0);
	i2c_lcdWriteStr(I2C1,PCF8574,own_ip);
	HAL_Delay(1000);

	/*load nilai Ro untuk kalibrasi sensor gas*/
	roTmp[0]= *(uint32_t*) add0;
	roTmp[1]= *(uint32_t*) add1;
	ro[0]= (float)roTmp[0]/10000;
	ro[1]= (float)roTmp[1]/10000;

	/*inisialisasi kompas dan peripheral tambahan*/
	reset_gy26(I2C1);
	HAL_Delay(100);
	for(int i=0;i<10;i++){
		refOrient= (get_angle(I2C1)%360);
		HAL_Delay(100);
	}
	HAL_Delay(100);
	UART2_InterruptReception();
	DMA1_Stream1_Init((uint32_t)&liBuff,(uint32_t)&USART3->DR,bSize);
	UART_DMA_reception(USART3);
	TIM3_Init();
	TIM6_Init();
	TIM4_Init();
	lidar_command(USART3,RP_START);

	/*program yang dieksekusi berulang-ulang*/
	while(1){
		if(calib_state==1){
			/*tampilan LCD untuk proses kalibrasi*/
			i2c_lcdClear(I2C1,PCF8574);
			i2c_lcdSetCursor(I2C1,PCF8574,0,2);
			i2c_lcdWriteStr(I2C1,PCF8574,"calibrating");
			i2c_lcdSetCursor(I2C1,PCF8574,1,3);
			i2c_lcdWriteStr(I2C1,PCF8574,"MQ sensor");
		}
		else if(calib_state==2){
			/*tampilan LCD selesai proses kalibrasi*/
			i2c_lcdClear(I2C1,PCF8574);
			i2c_lcdSetCursor(I2C1,PCF8574,0,4);
			i2c_lcdWriteStr(I2C1,PCF8574,"done..!!");
			HAL_Delay(1000);
			calib_state= 0;
			ro[0]= MQ_Calibration(roPol0,Ro_Factor,pol);
			ro[1]= MQ_Calibration(roPol1,Ro_Factor,pol);
			i2c_lcdClear(I2C1,PCF8574);
			i2c_lcdSetCursor(I2C1,PCF8574,0,0);
			sprintf(buff3,"Ro l= %.2fk",ro[0]);
			i2c_lcdWriteStr(I2C1,PCF8574,buff3);
			i2c_lcdSetCursor(I2C1,PCF8574,1,0);
			sprintf(buff3,"Ro r= %.2fk",ro[1]);
			i2c_lcdWriteStr(I2C1,PCF8574,buff3);

			/*simpan nilai Ro terkalibrasi ke eeprom*/
			HAL_FLASH_Unlock();
			FLASH_Erase_Sector(FLASH_SECTOR_11,VOLTAGE_RANGE_3);
			HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,add0,(uint32_t)(ro[0]*10000));
			HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,add1,(uint32_t)(ro[1]*10000));
			HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,add2,(uint32_t)(this_robot));
			HAL_FLASH_Lock();
			pol= 0;
		}
		else if(calib_state1==1){
			/*tampilan LCD untuk proses scanning heatmap gas*/
			i2c_lcdClear(I2C1,PCF8574);
			i2c_lcdSetCursor(I2C1,PCF8574,0,0);
			sprintf(buff3,"%4d| Scanning!",relativeOrient);
			i2c_lcdWriteStr(I2C1,PCF8574,buff3);
			i2c_lcdSetCursor(I2C1,PCF8574,1,0);
			sprintf(buff3,"x=%6dy=%6d",cartesianG[0],cartesianG[1]);
			i2c_lcdWriteStr(I2C1,PCF8574,buff3);
		}
		else if(calib_state1==2){
			/*proses scanning selesai*/
			i2c_lcdClear(I2C1,PCF8574);
			i2c_lcdSetCursor(I2C1,PCF8574,0,4);
			i2c_lcdWriteStr(I2C1,PCF8574,"Scanning");
			i2c_lcdSetCursor(I2C1,PCF8574,1,4);
			i2c_lcdWriteStr(I2C1,PCF8574,"done..!!");
			calib_state1=0;
			HAL_Delay(1000);
		}
		else{
			/*tampilan LCD default*/
			i2c_lcdClear(I2C1,PCF8574);
			sprintf(buff3,"%.1f",ppmK[0]);
			i2c_lcdSetCursor(I2C1,PCF8574,0,0);
			i2c_lcdWriteStr(I2C1,PCF8574,buff3);
			i2c_lcdSetCursor(I2C1,PCF8574,0,6);
			sprintf(buff3,"|%c%c|",warn,id1);
			i2c_lcdWriteStr(I2C1,PCF8574,buff3);
			sprintf(buff3,"%.1f",ppmK[1]);
			i2c_lcdSetCursor(I2C1,PCF8574,0,10);
			i2c_lcdWriteStr(I2C1,PCF8574,buff3);
			sprintf(buff3,"%.3fk",ro[0]);
			i2c_lcdSetCursor(I2C1,PCF8574,1,0);
			i2c_lcdWriteStr(I2C1,PCF8574,buff3);
			i2c_lcdSetCursor(I2C1,PCF8574,1,6);
			sprintf(buff3,"|%d%d|",commandState,pos);
			i2c_lcdWriteStr(I2C1,PCF8574,buff3);
			sprintf(buff3,"%.3fk",ro[1]);
			i2c_lcdSetCursor(I2C1,PCF8574,1,10);
			i2c_lcdWriteStr(I2C1,PCF8574,buff3);
		}

		/*mendapatkan orientasi robot*/
		actOrient= get_angle(I2C1)%360;
		simpangan= actOrient-refOrient;
		if(simpangan>= 180){
			simpangan= -1*(((360-actOrient)+refOrient)%360);
		}
		else if(simpangan<= -180){
			simpangan= ((360-refOrient)+actOrient)%360;
		}

		/*kalman filter untuk orientasi*/
		XpOrient= XeOrient;
		PpOrient= PeOrient+varProcOrient;
		GOrient= (float)PpOrient/(PpOrient+varOrient);
		XeOrient= XpOrient+(GOrient*(simpangan-XpOrient));
		PeOrient= (1-GOrient)*PpOrient;
		relativeOrient= XeOrient;
		HAL_Delay(200);
	}
}

/*timer 4 interrupt subroutine setiap 0.05 detik untuk sampling gas*/
void TIM4_IRQHandler(void){
	if(TIM4->SR&(1UL)){
		GPIOD->ODR^= (1UL<<12U);

		/*posisi robot tanpa metaheuristik*/
		if(!meta){
			pos= this_robot;
		}

		/*odometri robot*/
		if(pid[1]>=0){
			encoder[1]= 3*(TIM2->CNT);
		}
		else if(pid[1]<0){
			encoder[1]= -3*(TIM2->CNT);
		}
		if(pid[0]>=0){
			encoder[0]= 3*(TIM1->CNT);
		}
		else if(pid[0]<0){
			encoder[0]= -3*(TIM1->CNT);
		}
		int magV= (int)(encoder[0]+encoder[1])/2;
		cartesianG[0]= (cartesianG[0]+(int)(magV*sin(relativeOrient/PI)));
		cartesianG[1]= (cartesianG[1]+(int)(magV*cos(relativeOrient/PI)));
		cartesianG[2]= relativeOrient;
		actStep[0]= actStep[0]+magV;
		actStep[1]= relativeOrient;

		/*sampling gas*/
		adc[0]= ADC_getVal(ADC1);
		adc[1]= ADC_getVal(ADC2);
		rs[0]= MQ_Get_Resistance(adc[0],RLoad);
		rs[1]= MQ_Get_Resistance(adc[1],RLoad);
		ppm[0]= (float)MQ_Get_PPM((float)rs[0]/(ro[0]),curve);
		ppm[1]= (float)MQ_Get_PPM((float)rs[1]/(ro[1]),curve);
		/*pooling selama kalibrasi*/
		if(calib_state==1){
			roPol0[pol]=rs[0];
			roPol1[pol]=rs[1];
			pol++;
			if(pol>=nPol)calib_state++;
		}
		/*kalman filter gas*/
		for(int i=0;i<2;i++){
			Pc[i]= P[i]+varProc;
			G[i]= (float)Pc[i]/(Pc[i]+var[i]);
			P[i]= (1-G[i])*Pc[i];
			Xp[i]= Xe[i];
			Zp[i]= Xp[i];
			Xe[i]= (G[i]*(ppm[i]-Zp[i]))+Xp[i];
			ppmK[i]= Xe[i];
		}

		/*sampling data posisi robot dalam kartesian*/
		convert_cartesian(gama,teta,yval,xval);
		for(int i=0;i<2;i++){
			mea[i][0]=yval[i];
			mea[i][1]=xval[i];
			mea[i][2]=yval[i]-yval1[i];
			mea[i][3]=xval[i]-xval1[i];
		}

		/*kalman filter untuk posisi robot dalam kartesian*/
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

		for(int i=0;i<2;i++){
			for(int j=0;j<4;j++){
				if(j<2){
					Xp1[i][j]= Xe1[i][j]+(Xe1[i][j+2]/5);
				}
				else{
					Xp1[i][j]= Xe1[i][j];
				}
				Pp1[i][j]= Pe1[i][j]+varProc1[i][j];
			}
		}
		for(int i=0;i<2;i++){
			for(int j=0;j<4;j++){
				G1[i][j]= (float)Pp1[i][j]/(Pp1[i][j]+var1[i][j]);
				Xe1[i][j]= Xp1[i][j]+(G1[i][j]*(mea[i][j]-Xp1[i][j]));
				Pe1[i][i]= (1-G1[i][j])*Pp1[i][j];
			}
		}

		/*mengembalikan dari koordinat kartesian menuju polar*/
		for(int i=0;i<2;i++){
			yvalK[i]=Xe1[i][0];
			xvalK[i]=Xe1[i][1];
			if(yvalK[i]<0){
				if(yvalK[i]!=0){
					tetaK[i]= 180+(int)(PI*atan((float)xvalK[i]/yvalK[i]));
				}
				else{
					tetaK[i]= 180+(int)(PI*atan((float)xvalK[i]/(yvalK[i]+1)));
				}
			}
			else{
				if(yvalK[i]!=0){
					tetaK[i]= (360+(int)(PI*atan((float)xvalK[i]/yvalK[i])))%360;
				}
				else{
					tetaK[i]= (360+(int)(PI*atan((float)xvalK[i]/(yvalK[i]+1))))%360;
				}
			}
			gamaK[i]= sqrt(pow(xvalK[i],2)+pow(yvalK[i],2));
			tetaK[i]= (tetaK[i]+180)%360-180;
		}

		/*shift nilai pada akhir subroutine*/
		for(int i=0;i<2;i++){
			yval1[i]= yvalK[i];
			xval1[i]= xvalK[i];
			teta1[i]= tetaK[i];
			gama1[i]= gamaK[i];
		}

		TIM2->EGR|= (1UL);
		TIM1->EGR|= (1UL);

		TIM4->SR&= ~(1UL);
	}
}

/*DMA interrupt ketika LiDAR telah selesai scanning*/
void DMA1_Stream1_IRQHandler(void){
	if(DMA1->LISR &(1UL<<11U)){
		/*stop lidar*/
		lidar_command(USART3,RP_STOP);
		GPIOD->BSRR|= (1UL<<14U);
		/*konversi data mentah lidar menjadi koordinat polar*/
		rplidar_conversion(liBuff,bSize,limit,dist,tim5state);
		probabilistic_filter(dist,dist1,1,6,3);

		/*mendapatkan posisi kawanan robot*/
		int stdev,mean;
		derivate_graph(dist1,ddist,360,&stdev,&mean);
		find_robot(ddist,dist1,stdev,mean,360,1,shadow_teta,shadow_gama);
		find_theBest(shadow_gama,shadow_teta,360,teta1,gama1,teta,gama,limit,tim5state);

		/*menghapus temporary variable*/
		memset(liBuff,0,sizeof(liBuff));
		memset(shadow_gama,0,sizeof(shadow_gama));
		memset(shadow_teta,0,sizeof(shadow_teta));
		GPIOD->BSRR|= (1UL<<30U);

		if(!tim5state){
			tim5state=1;
			TIM5_Init();
		}

		DMA1->LIFCR|= (1UL<<11U);
	}
}

/*timer 5 interrupt digunakan untuk mengirimkan perintah ke lidar untuk start scanning*/
void TIM5_IRQHandler(void){
	if(TIM5->SR&(1UL)){
		GPIOD->ODR^= (1UL<<13U);
		/*lidar start scanning*/
		lidar_command(USART3,RP_START);

		/*fuzzy logic dan perebutan posisi dieksekusi dalam perioda 1 detik*/
		relativeOrient_step= relativeOrient-relativeOrient1;
		measure_velocity(gamaK,gama2,relativeOrient_step,tetaK,teta2,veloMag,veloDir,tim5state);

		/*perebutan formasi*/
		if(meta){
			fn[0]= (int)actStep[0];
			fn[1]= (int)(((gamaK[0]/10)*cos(((tetaK[0]+relativeOrient-veloDir[0])%360)/PI))+(veloMag[0]));
			fn[2]= (int)(((gamaK[1]/10)*cos(((tetaK[1]+relativeOrient-veloDir[1])%360)/PI))+(veloMag[1]));
			pos= metaheuristicGetpos_and_cartDest(id1,ID,relativeOrient,fn,veloDir,tetaK,gamaK,bentukForm,bestOther,cartDestinationDist);
		}
		else{
			int tmpVal[2];
			for(int i=0;i<2;i++){
				tmpVal[i]= gamaK[i]*cos(((tetaK[i]+relativeOrient)%360)/PI);
			}

			if(tmpVal[0]>tmpVal[1]){
				bestOther[0]= tetaK[0];
				bestOther[1]= gamaK[0];
				arahBestOther= veloDir[0];
			}
			else{
				bestOther[0]= tetaK[1];
				bestOther[1]= gamaK[1];
				arahBestOther= veloDir[1];
			}
			get_cartDest(pos,relativeOrient,arahBestOther,bentukForm,bestOther,cartDestinationDist);
		}
		get_vector_distance(pos,cartDestinationDist,vektor);

		/*fuzzy logic*/
		/*gas*/
		for(int i=0;i<2;i++){
			tmpPPM[i]=ppmK[i];
			dppmK= gasFuzzIn[0];
		}
		gasFuzzIn[0]= sqrt(pow(tmpPPM[0],2)+pow(tmpPPM[1],2));
		gasFuzzIn[1]= (tmpPPM[0]-ppmK1[0])-(tmpPPM[1]-ppmK1[1]);
		gasFuzzIn[2]= gasFuzzIn[0]-dppmK;

		uppm[0]= fs_trapesium_sikukiri(gasFuzzIn[0],0,40.5,46.5,100);			//resultan konsentrasi gas
		uppm[1]= fs_segitiga(gasFuzzIn[0],40.5,65.5,85.0,100);
		uppm[2]= fs_trapesium_sikukanan(gasFuzzIn[0],80.0,120,20000,100);
		uselisih[0]= fs_trapesium_sikukiri(gasFuzzIn[1],-20000,-3.60,-2.0,100);	//selisih perubahan konsentrasi gas sensor kanan dan kiri
		uselisih[1]= fs_segitiga(gasFuzzIn[1],-2.5,-1.45,-1.1,100);
		uselisih[2]= fs_segitiga(gasFuzzIn[1],-1.25,-0.95,-0.28,100);
		uselisih[3]= fs_segitiga(gasFuzzIn[1],-0.5,0,0.5,100);
		uselisih[4]= fs_segitiga(gasFuzzIn[1],0.28,0.95,1.25,100);
		uselisih[5]= fs_segitiga(gasFuzzIn[1],1.1,1.45,2.5,100);
		uselisih[6]= fs_trapesium_sikukanan(gasFuzzIn[1],2.0,3.60,20000,100);
		udppm[0]= fs_trapesium_sikukiri(gasFuzzIn[2],-20000,-5.5,-0.75,100);	//perubahan konsentrasi
		udppm[1]= fs_segitiga(gasFuzzIn[2],-1.0,0,1.0,100);
		udppm[2]= fs_trapesium_sikukanan(gasFuzzIn[2],0.75,5.5,20000,100);
		inference(uppm,udppm,uselisih,tmpInference);
		center_area(tmpInference,63,ruleGas,GasParam);

		if(pos!= 1){
			if(pos==2){
				if(GasParam[1]>0)GasParam[1]= 0;
			}
			else if(pos==3){
				if(GasParam[1]<0)GasParam[1]= 0;
			}
		}

		/*formasi*/
		lidarFuzzIn[0]= vektor[0];
		lidarFuzzIn[1]= vektor[1]-relativeOrient;
		lidarFuzzIn[2]= vektor[0]-mag1;
		lidarFuzzIn[3]= lidarFuzzIn[1]-pol1;

		uEmag1[0]= fs_trapesium_sikukiri(lidarFuzzIn[0],0,50,70,100);			//error magnitude jarak
		uEmag1[1]= fs_segitiga(lidarFuzzIn[0],60,150,225,100);
		uEmag1[2]= fs_trapesium_sikukanan(lidarFuzzIn[0],200,400,3000,100);
		uEpol1[0]= fs_trapesium_sikukiri(lidarFuzzIn[1],-90,-12,-3,100);		//error arah
		uEpol1[1]= fs_segitiga(lidarFuzzIn[1],-5,0,5,100);
		uEpol1[2]= fs_trapesium_sikukanan(lidarFuzzIn[1],3,12,90,100);
		udEmag1[0]= fs_trapesium_sikukiri(lidarFuzzIn[2],-200,-100,-30,100);	//error perubahan jarak
		udEmag1[1]= fs_trapesium_samakaki(lidarFuzzIn[2],-40,-10,10,40,100);
		udEmag1[2]= fs_trapesium_sikukanan(lidarFuzzIn[2],30,100,200,100);
		udEpol1[0]= fs_trapesium_sikukiri(lidarFuzzIn[3],-90,-20,-5,100);		//error perubahan arah
		udEpol1[1]= fs_trapesium_samakaki(lidarFuzzIn[3],-7,-2,2,7,100);
		udEpol1[2]= fs_trapesium_sikukanan(lidarFuzzIn[3],5,20,90,100);
		inference_lidar1(uEmag1,uEpol1,udEpol1,udEmag1,tmpInferenceLidar1);
		center_area_lidar1(tmpInferenceLidar1,81,ruleLidar1,LidarParam);

		/*behavior-based formation control*/
		step[0]= (matrice[0]*GasParam[0])+(matrice[1]*LidarParam[0]);
		step[1]= step[1]+(matrice[0]*GasParam[1])+(matrice[1]*LidarParam[1]);
		if(step[1]>= 90){
			step[1]= 90;
		}
		else if(step[1]<= -90){
			step[1]= -90;
		}

		if(!calib_state1){
			stepSP[0]=step[0];
			stepSP[1]=step[1];
			/*send to server setial 1 detik untuk monitoring*/
			sprintf(buff4,"%c;%c;%6d;%6d;%4d;%4d;%4d;%4d;%4d;%4d;%4d;%9d;%9d;%4d;%4d;%4d;%4d;%4d;%4d;%4d;%1d;%4d;%4d;%4d;%5d;%5d"
						   ,'A',id1,(int)(ppmK[0]*10),(int)(ppmK[1]*10),relativeOrient,stepSP[0],stepSP[1],actStep[0],pid[0],pid[1],cartesianG[0],cartesianG[1],tetaK[0],gamaK[0],tetaK[1],gamaK[1],fn[0],fn[1],fn[2],pos,veloMag[0],veloMag[1],veloDir[0],veloDir[1],cartDestinationDist[0],cartDestinationDist[1]);
			send_udp(USART2,ID,buff4);

			/*kirim untuk menghitung nilai fitness*/
			sprintf(buff4,"%c;%c"
						   ,'B',id1);
			send_udp(USART2,ID,buff4);
		}
		else{
			stepSP[0]= 50;
			sprintf(buff4,"%d;%d;%d",cartesianG[1],cartesianG[0],(int)(gasFuzzIn[0]*10));
			send_udp(USART2,ID,buff4);
			if(cartesianG[1]>= 3000){
				TIM6->CR1&= ~(1UL);
				TIM3->CR1&= ~(1UL);
			}
		}

		/*step reset after fuzzy process*/
		actStep[0]=0;
		for(int i=0;i<2;i++){
			dirI[i]=0;
			dirD[i]=0;
			dirE1[i]=0;
			ppmK1[i]=tmpPPM[i];
		}

		relativeOrient1= relativeOrient;
		for(int i=0;i<2;i++){
			teta2[i]=tetaK[i];
			gama2[i]=gamaK[i];
		}

		mag1=lidarFuzzIn[0];
		pol1=lidarFuzzIn[1];

		TIM5->SR&= ~(1UL);
	}
}

/*external interrupt untuk kalibrasi Ro*/
void EXTI0_IRQHandler(void){
	if(EXTI->PR&(1UL)){
		calib_state= calib_state+1;
		calib_state1=0;
		EXTI->PR|= (1UL);
	}
}

/*external interrupt untuk tambah koordinat X*/
void EXTI1_IRQHandler(void){
	if(EXTI->PR&(2UL)){
		if(cartesianG[0]<= 3000){
			cartesianG[0]=cartesianG[0]+300;	//cartesian increment 30cm
			cartesianG[1]= 0;
		}
		else{
			calib_state1++;
		}
		EXTI->PR|= (1UL<<1U);
	}
}

/*external interrupt untuk mode scanning*/
void EXTI2_IRQHandler(void){
	if(EXTI->PR&(4UL)){
		if(!calib_state1){
			calib_state1++;						//state untuk mode scanning
			calib_state=0;
			cartesianG[0]=0;
			cartesianG[1]=0;
		}
		EXTI->PR|= (1UL<<2U);
	}
}

/*setting ID robot*/
void EXTI3_IRQHandler(void){
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
		/*simpan ID baru*/
		HAL_FLASH_Unlock();
		FLASH_Erase_Sector(FLASH_SECTOR_11,VOLTAGE_RANGE_3);
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,add0,(uint32_t)(ro[0]*10000));
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,add1,(uint32_t)(ro[1]*10000));
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,add2,(uint32_t)(this_robot));
		HAL_FLASH_Lock();

		EXTI->PR|= (1UL<<3U);
	}
}

/*timer 6 untuk PID kontrol kecepatan roda kanan dan kiri*/
void TIM6_DAC_IRQHandler(void){
	if(TIM6->SR&(1UL)){

		for(int i=0;i<2;i++){
			dirE[i]= stepSP[i]-actStep[i];
		}
		for(int i=0;i<2;i++){
			dirI[i]= dirI[i]+dirE[i];
			dirD[i]= dirE1[i]-dirE[i];
		}

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

		motor_jalan(TIM3,KIRI,pid[0]);
		motor_jalan(TIM3,KANAN,pid[1]);

		TIM6->SR&= ~(1UL);
	}
}

/*USART interrup menerima perintah dari ground station*/
void USART2_IRQHandler(void){
	if(USART2->SR & (1UL<<5U)){
		char uart_temp= USART2->DR;

		if(uart_temp== '&'){
			idx1= 0;
			buffTerima[idx1]= uart_temp;
			stateTerima= 1;
		}
		else if(stateTerima){
			idx1++;
			buffTerima[idx1]= uart_temp;
			if(uart_temp== '#'){
				idx1= 0;
				stateTerima= 0;
				sscanf(buffTerima,"&%d#",&commandState);
			}
		}

		switch(commandState){
		case 0:{
			meta= 0;
			pid[0]= 0;
			pid[1]= 0;
			matrice[0]= 0;
			matrice[1]= 0;
			TIM3->CR1&= ~(1UL);
			TIM6->CR1&= ~(1UL);
		}
		break;
		case 1:{
			meta= 0;
			matrice[0]= 0;
			matrice[1]= 1;
			TIM3->CR1|= (1UL);
			TIM6->CR1|= (1UL);
		}
		break;
		case 2:{
			meta= 0;
			matrice[0]= 1;
			matrice[1]= 0;
			TIM3->CR1|= (1UL);
			TIM6->CR1|= (1UL);
		}
		break;
		case 3:{
			meta= 0;
			matrice[0]= 1;
			matrice[1]= 1;
			TIM3->CR1|= (1UL);
			TIM6->CR1|= (1UL);
		}
		break;
		case 4:{
			meta= 1;
			matrice[0]= 1;
			matrice[1]= 1;
			TIM3->CR1|= (1UL);
			TIM6->CR1|= (1UL);
		}
		break;
		}
		USART2->SR&= ~(1UL<<5U);
	}
}
