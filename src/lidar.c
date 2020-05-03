#include "lidar.h"

void lidar_command(USART_TypeDef* UARTx,uint8_t command){
	char temp[33];
	sprintf(temp,"%c%c",LIDARCOMMAND,command);
	UART_sendStr(UARTx,temp);
}

void rplidar_conversion(uint8_t *data_in,int data_inSize,uint16_t limit,uint16_t *data_out,int state){
	uint16_t pos= 0;
	while(data_in[pos]!= 0x3E)pos++;
	for(int i= pos;i<data_inSize;i++){
		if(data_in[i]==0x3E){
			uint16_t alpha= (180+((data_in[i+2]<<7)+(data_in[i+1]>>1))/64)%360;
			uint16_t beta= ((data_in[i+4]<<8)+data_in[i+3])/4;
			if(beta>= limit){
				beta= limit;
				data_out[alpha]= beta;
			}
			else if((beta<100)&(state==1)){
				int j=0;
				while(data_out[(alpha+(360-j))%360]<100){
					j++;
				}
				beta= data_out[(alpha+(360-j))%360];//data_in[(360+(i-1))%360];
				data_out[alpha]= beta;
			}
			else if((beta>=100) & (beta<limit)){
				data_out[alpha]= beta;
			}
		}
	}
}

void probabilistic_filter(uint16_t *input,uint16_t *output,float Gain,int sample,int overlap){
	for(int i=0;i<360;i=i+overlap){
		//find its mean and standar dev
		uint16_t tmp[sample];
		int sum= 0;
		for(int j=0;j<sample;j++){
			tmp[j]= input[(i+j)%360];
			sum= sum+tmp[j];
		}
		int mean= sum/sample;

		sum= 0;
		for(int j=0;j<sample;j++){
			sum= sum+ pow(mean-tmp[j],2);
		}
		int stdev= sqrt(sum/(sample-1));

		int sort;
		for(int j=0;j<sample;j++){
			for(int k=0;k<sample-1;k++){
				if(tmp[k]>tmp[k+1]){
					sort= tmp[k];
					tmp[k]= tmp[k+1];
					tmp[k+1]= sort;
				}
			}
		}
		//change the outlier with possible values
		for(int j=0;j<sample;j++){
			if(abs(input[(i+j)%360]-mean)>(int)(Gain*stdev)){
				output[(i+j)%360]= tmp[sample/2];
			}
			else{
				output[(i+j)%360]= input[(i+j)%360];
			}

			if(output[(i+j+360)%360]==0){
				output[(i+j+360)%360]= output[(i+j+359)%360];
			}
		}
	}
}

void derivate_graph(uint16_t *input,int *output,int size,int* stdev,int* mean){	//bisa int* stdev dan int* mean. di variable tulis *mean dan *stdev
	int sum= 0;
	for(int i=0;i<size;i++){
		output[i]= input[i%size]-input[(i+1)%size];
		sum= sum+output[i];
	}
	*mean= sum/360;

	sum=0;
	for(int i=0;i<360;i++){
		sum= sum+ pow(*mean-output[i],2);
	}
	*stdev= sqrt(sum/360);
}

void find_robot(int *derivative,uint16_t *distance,int stdev,int mean,int size,float gain,int* angle,uint16_t *dist){
	int i=0;
	int number= 0;
	while(i< size){
		if((abs(derivative[i%360]-mean)> (int)(gain*stdev))&&(derivative[i%360]> 0)){
			int j=0;
			while(!((abs(derivative[(i+j)%360]-mean)> (int)(gain*stdev))&&(derivative[(i+j)%360]< 0))){
				j++;
				if(j> 45){
					break;
				}
			}

			if((j<= 45)&&(j>=3)&&(distance[(i+(j/2))%360]> 100)){
				angle[number]= (i+(j/2)+1)%360;
				dist[number]= distance[(i+(j/2)+1)%360];
				number++;
				i= i+j+10;
			}
			else{
				i++;
			}
		}
		else{
			i++;
		}
	}
}

void find_theBest(uint16_t *input,int *input1,int size,int *measured_angle,int *measured_dist,int *out,uint16_t *out1,int limit_distance,int state){
	int n=0;
	int tmpout_ang[2];
	int tmpout_dis[2];

	while((input[n]!=0)&(n<360)){
		n++;
	}

	if(n>=2){
		int temp_angle[n];
		uint16_t temp_jarak[n];

		for(int i=0;i<n;i++){
			temp_angle[i]= input1[i];
			temp_jarak[i]= input[i];
		}

		uint16_t sort;
		int sort1;
		for(int i=0;i<n;i++){
			for(int j=0;j<n-1;j++){
				if(temp_jarak[j]>temp_jarak[j+1]){
					sort= temp_jarak[j];
					sort1= temp_angle[j];
					temp_jarak[j]=temp_jarak[j+1];
					temp_angle[j]=temp_angle[j+1];
					temp_jarak[j+1]= sort;
					temp_angle[j+1]=sort1;
				}
			}
		}

		for(int i=0;i<2;i++){
			if((temp_jarak[i]>=100)&(temp_jarak[i]<=limit_distance)&(state==0)){
				tmpout_ang[i]= temp_angle[i];
				tmpout_dis[i]= temp_jarak[i];
			}
			else if((temp_jarak[i]>=100)&(temp_jarak[i]<=limit_distance)&(state==1)){
				tmpout_ang[i]= temp_angle[i];
				tmpout_dis[i]= temp_jarak[i];
			}
			else if(((temp_jarak[i]<100)|(temp_jarak[i]>limit_distance))&(state==1)){
				tmpout_ang[i]= measured_angle[i];
				tmpout_dis[i]= measured_dist[i];
			}
		}
	}
	else{
		for(int i=0;i<2;i++){
			tmpout_ang[i]= measured_angle[i];
			tmpout_dis[i]= measured_dist[i];
		}
	}

	if(state){
		//euclidean dist
		for(int i=0;i<2;i++){
			int edist[2];
			for(int j=0;j<2;j++){
				edist[j]=0;
				int delta1= 0;
				if(state){
					delta1= abs(tmpout_ang[i]-measured_angle[j]);
					if(delta1> 180){
						delta1= abs((360-tmpout_ang[i])+measured_angle[j]);
						if(delta1>180){
							delta1= abs((360-measured_angle[j])+tmpout_ang[i]);
							}
						}
					}
				edist[j]= sqrt(pow(delta1,2)+pow(tmpout_dis[i]-measured_dist[j],2));
			}

			int pos=0;
			int minVal= edist[0];
			for(int j=0;j<2;j++){
				if(edist[j]<minVal){
					minVal= edist[j];
					pos= j;
				}
			}
			if(minVal<= 600){
				out[i]= tmpout_ang[pos];
				out1[i]= (uint16_t)tmpout_dis[pos];
			}
			else{
				out[i]= measured_angle[i];
				out1[i]= (uint16_t)measured_dist[i];
			}
		}
	}
	else{
		for(int i=0;i<2;i++){
			out[i]= tmpout_ang[i];
			out1[i]= tmpout_dis[i];
		}
	}
}

void measure_velocity(int *pos,int *pos1,int orientasi,int *angle,int *angle1,int *velocity,int *direction,int state){
	int velox[2],veloy[2];
	if(state){
		for(int i=0;i<2;i++){
			veloy[i]=(int)(pos[i]*cos((angle[i]-orientasi)/PI)-pos1[i]*cos(angle1[i]/PI));
			velox[i]=(int)(pos[i]*sin((angle[i]-orientasi)/PI)-pos1[i]*sin(angle1[i]/PI));
			if(veloy[i]>= 0){
				velocity[i]= sqrt(pow(velox[i],2)+pow(veloy[i],2));
			}
			else{
				velocity[i]= -1*(sqrt(pow(velox[i],2)+pow(veloy[i],2)));
			}

			if((veloy[i]<0)&(velox[i]<0)){
				direction[i]= (int)(PI*atan((float)velox[i]/(veloy[i]+1)))+180;
			}
			else if((veloy[i]<0)&(velox[i]>=0)){
				direction[i]= 180+(int)(PI*atan((float)velox[i]/(veloy[i]+1)));
			}
			else if((veloy[i]>=0)&(velox[i]<0)){
				direction[i]= 360+(int)(PI*atan((float)velox[i]/(veloy[i]+1)));
			}
			else{
				direction[i]= (int)(PI*atan((float)velox[i]/(veloy[i]+1)));
			}
		}
	}
	else{
		for(int i=0;i<2;i++){
			velocity[i]= 0;
			direction[i]= 0;
		}
	}
}

void convert_cartesian(uint16_t *dist,int *angle,int *y_axis,int *x_axis){
	for(int i=0;i<2;i++){
		y_axis[i]= dist[i]*cos(angle[i]/PI);
		x_axis[i]= dist[i]*sin(angle[i]/PI);
	}
}

uint16_t get_distance(uint16_t *lidar_scan,int start,int end){
	uint16_t minVal= lidar_scan[start];
	for(int i= start;i<end;i++){
		if((lidar_scan[i]<= minVal)&(lidar_scan[i]>= 150)){
			minVal= lidar_scan[i];
		}
	}
	return minVal;
}
