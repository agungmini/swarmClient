/*
 * particle_swarm.c
 *
 *  Created on: Jun 14, 2020
 *      Author: nursyeha
 */

#include "particle_swarm.h"

char buff5[33];
																				//sudut		//jarak
int metaheuristicGetpos_and_cartDest(char ida,int ide,int orient,int *fitness,int *direction,int *pos1,int *pos2,int *form_shape,int *best,int *cartDest){
	int returnVal;

	if(fitness[0]>fitness[1] && fitness[0]>fitness[2]){
		returnVal= 1;

		cartDest[0]= 0;
		cartDest[1]= 0;
		best[0]= 0;
		best[1]= 0;
	}
	else{
		int posfnMax[2][2];
		int direct;
		if(fitness[1]>fitness[2]){
			posfnMax[0][0]=pos1[0];		//sudut terbaik
			posfnMax[0][1]=pos2[0];		//jarak terbaik
			direct= direction[0];
			posfnMax[1][0]= pos1[1];
			posfnMax[1][1]= pos2[1];
		}
		else{
			posfnMax[0][0]=pos1[1];		//sudut terbaik
			posfnMax[0][1]=pos2[1];		//jarak terbaik
			direct= direction[1];
			posfnMax[1][0]= pos1[0];
			posfnMax[1][1]= pos2[0];
		}

		int cart[2][2],l2[2],l3[2],a[2],b[2];
		int someVal[2];
		someVal[0]= form_shape[0]*sin((form_shape[1]+orient-direct)/PI);
		someVal[1]= form_shape[0]*cos((form_shape[1]+orient-direct)/PI);

		//posisi tujuan untuk pos 2
		cart[0][0]= posfnMax[0][1]*cos((posfnMax[0][0]+orient-direct)/PI)-someVal[0];
		cart[0][1]= posfnMax[0][1]*sin((posfnMax[0][0]+orient-direct)/PI)-someVal[1];
		//posisi tujuan untuk pos 3
		cart[1][0]= posfnMax[0][1]*cos((posfnMax[0][0]+orient-direct)/PI)-someVal[0];
		cart[1][1]= posfnMax[0][1]*sin((posfnMax[0][0]+orient-direct)/PI)+someVal[1];

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
			returnVal= 2;
		}
		else if(b[0]<a[0]){
			returnVal= 3;
		}
		else{
			if(a[1]<b[1]){
				returnVal= 2;
			}
			else{
				returnVal= 3;
			}
		}

		best[0]= posfnMax[0][0];
		best[1]= posfnMax[0][1];
		if(returnVal== 2){
			cartDest[1]= cart[0][0]; //y
			cartDest[0]= cart[0][1]; //x
		}
		else if(returnVal== 3){
			cartDest[1]= cart[1][0]; //y
			cartDest[0]= cart[1][1]; //x
		}
	}

	return returnVal;
}

void get_cartDest(int posisi,int orient,int direct,int *form_shape,int *bestPos,int *cartDist){
	int someVal[2];
	someVal[0]= form_shape[0]*sin((form_shape[1]+orient-direct)/PI);
	someVal[1]= form_shape[0]*cos((form_shape[1]+orient-direct)/PI);

	//posisi tujuan untuk pos 1
	if(posisi==1){
		someVal[0]= form_shape[0]*sin((form_shape[1])/PI);
		someVal[1]= form_shape[0]*cos((form_shape[1])/PI);
		if(bestPos[0]>= 0){
			if(bestPos[0]<= 90){cartDist[1]= someVal[0]-bestPos[1]*cos((bestPos[0]+orient)/PI);}	//y
			else{cartDist[1]= someVal[0]+bestPos[1]*cos((bestPos[0]+orient)/PI);}					//y
			cartDist[0]= bestPos[1]*sin((bestPos[0]+orient)/PI)-someVal[1];							//x
		}
		else{
			if(bestPos[0]<= 90){cartDist[1]= someVal[0]-bestPos[1]*cos((bestPos[0]+orient)/PI);}	//y
			else{cartDist[1]= someVal[0]+bestPos[1]*cos((bestPos[0]+orient)/PI);}					//y
			cartDist[0]= someVal[1]-bestPos[1]*sin((bestPos[0]+orient)/PI);							//x
		}
	}
	//posisi tujuan untuk pos 2
	else if(posisi==2){
		cartDist[1]= bestPos[1]*cos((bestPos[0]+orient-direct)/PI)-someVal[0];	//y
		cartDist[0]= bestPos[1]*sin((bestPos[0]+orient-direct)/PI)-someVal[1];	//x
	}
	//posisi tujuan untuk pos 3
	else if(posisi==3){
		cartDist[1]= bestPos[1]*cos((bestPos[0]+orient-direct)/PI)-someVal[0];	//y
		cartDist[0]= bestPos[1]*sin((bestPos[0]+orient-direct)/PI)+someVal[1];	//x
	}
}
