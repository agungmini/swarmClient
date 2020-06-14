/*
 * particle_swarm.c
 *
 *  Created on: Jun 14, 2020
 *      Author: nursyeha
 */

#include "particle_swarm.h"

char buff5[33];
													//sudut		//jarak
int metaheuristicGetpos(int orient,int *fitness,int *pos1,int *pos2){
	if(fitness[0]>fitness[1] && fitness[0]>fitness[2]){
		//sprintf(buff5,"terbesar");
		//send_udp(USART2,3,buff5);
		return 1;
	}
	else{
		int posfnMax[2][2];
		if(fitness[1]>fitness[2]){
			posfnMax[0][0]=pos1[0];
			posfnMax[0][1]=pos2[0];
			posfnMax[1][0]= pos1[1];
			posfnMax[1][1]= pos2[1];
		}
		else{
			posfnMax[0][0]=pos1[1];
			posfnMax[0][1]=pos2[1];
			posfnMax[1][0]= pos1[0];
			posfnMax[1][1]= pos2[0];
		}

		//sprintf(buff5,"[%d %d %d %d]",posfnMax[0][0],posfnMax[0][1],posfnMax[1][0],posfnMax[1][1]);
		//send_udp(USART2,3,buff5);

		int cart[2][2],l2[2],l3[2],a[2],b[2];
		//posisi tujuan untuk pos 2
		cart[0][0]= posfnMax[0][1]*cos((posfnMax[0][0]+orient)/PI)-712;
		cart[0][1]= posfnMax[0][1]*sin((posfnMax[0][0]+orient)/PI)-500;
		//posisi tujuan untuk pos 3
		cart[1][0]= posfnMax[0][1]*cos((posfnMax[0][0]+orient)/PI)-712;
		cart[1][1]= posfnMax[0][1]*sin((posfnMax[0][0]+orient)/PI)+500;

		//perebutan posisi 2
		l2[0]= sqrt(pow(cart[0][0],2)+pow(cart[0][1],2));
		l2[1]= sqrt(pow(cart[0][0]-(posfnMax[1][1]*cos((posfnMax[1][0]+orient)/PI)),2)+pow(cart[0][1]-(posfnMax[1][1]*sin((posfnMax[1][0]+orient)/PI)),2));
		if(l2[0]<l2[1]){
			a[0]= 0;
			a[1]= l2[0];
		}
		else{
			a[0]= 1;
			a[1]= l2[1];
		}

		//perebutan posisi 3
		l3[0]= sqrt(pow(cart[1][0],2)+pow(cart[1][1],2));
		l3[1]= sqrt(pow(cart[1][0]-(posfnMax[1][1]*cos((posfnMax[1][0]+orient)/PI)),2)+pow(cart[1][1]-(posfnMax[1][1]*sin((posfnMax[1][0]+orient)/PI)),2));
		if(l3[0]<l3[1]){
			b[0]= 0;
			b[1]= l3[0];
		}
		else{
			b[0]= 1;
			b[1]= l3[1];
		}

		if(a[0]<b[0]){
			return 2;
		}
		else if(b[0]<a[0]){
			return 3;
		}
		else{
			if(a[1]<b[1]){
				return 2;
			}
			else{
				return 3;
			}
		}

		//sprintf(buff5,"[%d %d][%d %d]",l2[0],l2[1],l3[0],l3[1]);
		//send_udp(USART2,3,buff5);
	}
}
