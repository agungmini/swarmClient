/*
 * fuzzy.c
 *
 *  Created on: Nov 29, 2019
 *      Author: nursyeha
 */

#include <fuzzy.h>

int fs_segitiga(float x,float lbound,float max,float hbound,int Val){
	if((x>=lbound) & (x<max)){
		return (int)(Val*(float)((x-lbound)/(max-lbound)));
	}
	else if((x>=max) & (x<hbound)){
		return (int)(Val*(float)((hbound-x)/(hbound-max)));
	}
	else{
		return 0;
	}
}

int fs_trapesium_samakaki(float x,float lbound,float lmax,float hmax,float hbound,int Val){
	if((x>=lbound) & (x<lmax)){
		return (int)(Val*((float)(x-lbound)/(lmax-lbound)));
	}
	else if((x>=lmax) & (x<hmax)){
		return Val;
	}
	else if((x>=hmax) & (x<hbound)){
		return (int)(Val*(float)((hbound-x)/(hbound-hmax)));
	}
	else{
		return 0;
	}
}

int fs_trapesium_sikukiri(float x,float lbound,float hmax,float hbound,int Val){
	if((x>=lbound) & (x<hmax)){
		return Val;
	}
	else if((x>=hmax) & (x<hbound)){
		return (int)(Val*(float)((hbound-x)/(hbound-hmax)));
	}
	else{
		return 0;
	}
}

int fs_trapesium_sikukanan(float x, float lbound,float lmax,float hbound,int Val){
	if((x>=lbound) & (x<lmax)){
		return (int)(Val*(float)((x-lbound)/(lmax-lbound)));
	}
	else if((x>=lmax) & (x<hbound)){
		return Val;
	}
	else{
		return 0;
	}
}

int fs_kotak(int x,int lower,int upper,int Val){
	if((x> lower)&(x<=upper)){
		return Val;
	}
	else{
		return 0;
	}
}

void inference(int param1[][3],int param2[][3],int *output){
	int x=0;
	for(int i=0;i<3;i++){
		for(int j=0;j<3;j++){
			for(int k=0;k<3;k++){
				for(int l=0;l<3;l++){
					output[x]=0;
					if((param1[0][i]!=0)&&(param1[1][j]!=0)&&(param2[0][k]!=0)&&(param2[1][l]!=0)){
						int a=0;
						int b=0;

						if(param1[0][i]<param1[1][j]){a= param1[0][i];}
						else{a= param1[1][j];}

						if(param2[0][k]<param2[1][l]){b= param2[0][k];}
						else{b= param2[1][l];}

						if(a<b){output[x]= a;}
						else{output[x]= b;}
					}
					x++;
				}
			}
		}
	}
}

void center_area(int *inference_result,int size,int rule[][81],int *output){
	int miu=0;
	int mius[2];
	mius[0]=0;
	mius[1]=0;
	for(int i=0;i<size;i++){
		mius[0]= mius[0]+(inference_result[i]*rule[0][i]);
		mius[1]= mius[1]+(inference_result[i]*rule[1][i]);
		miu=miu+inference_result[i];
	}
	output[0]= mius[0]/miu;
	output[1]= mius[1]/miu;
}

void inference_lidar(int *param1,int *param2,int *output){
	int x=0;
	for(int i=0;i<3;i++){
		for(int j=0;j<3;j++){
			output[x]=0;
			if((param1[i]!=0)&&(param2[j]!=0)){
				if(param1[i]<param2[j]){
					output[x]= param1[i];
				}
				else{
					output[x]= param2[j];
				}
			}
			x++;
		}
	}
}

void center_area_lidar(int pos,int *inference_result,int size,int rule[][2][9],int *output){
	int miu;
	int mius[2];
	miu=0;
	mius[0]=0;
	mius[1]=0;
	for(int i=0;i<size;i++){
		mius[0]=mius[0]+(inference_result[i]*rule[pos][0][i]);
		mius[1]=mius[1]+(inference_result[i]*rule[pos][1][i]);
		miu=miu+inference_result[i];
	}
	output[0]= mius[0]/miu;
	output[1]= mius[1]/miu;
}

void inference_avoidance(int *left,int *right,int *output_inference){
	int x=0;
	for(int i=0;i<3;i++){
		for(int j=0;j<3;j++){
			output_inference[x]=0;
			if((left[i]!=0)&&(right[j]!=0)){
				if(left[i]< right[j]){
					output_inference[x]= left[i];
				}
				else{
					output_inference[x]= right[j];
				}
			}
			x++;
		}
	}
}

int center_area_avoidance(int *inference_result,int size,int *rule){
	int miu=0;
	int mius=0;
	for(int i=0;i<size;i++){
			mius= mius+(inference_result[i]*rule[i]);
			miu= miu+inference_result[i];
	}
	return (mius/miu);
}

void inference_formation(int param[][4],int *output_inference){
	int x=0;
	for(int i=0;i<4;i++){
		for(int j=0;j<4;j++){
			output_inference[x]=0;
			if((param[0][i]!=0)&&(param[1][j]!=0)){
				if(param[0][i]< param[1][j]){
					output_inference[x]= param[0][i];
				}
				else{
					output_inference[x]= param[1][j];
				}
			}
			x++;
		}
	}
}

int center_area_formation(int *inference_result,int size,int *rule){
	int miu=0;
	int mius=0;
	for(int i=0;i<size;i++){
			mius= mius+(inference_result[i]*rule[i]);
			miu= miu+inference_result[i];
	}
	return (mius/miu);
}
