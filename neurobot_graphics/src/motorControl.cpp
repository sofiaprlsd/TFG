/*
 * velocityControl.c
 *
 *  Created on: Mar 4, 2024
 *      Author: juan
 */

#include "main/motorControl.h"
#include <math.h>



void initControlStructure(double K, double zeroPosition, struct controlData *a){
	a->K=K;
	a->zeroPosition=zeroPosition;
	return;
}

double applySpeedControl(struct controlData *a){
	a->input[1]=a->input[0];
	a->input[0]=a->error;
	a->output[1]=a->output[0];
	a->output[0]=a->K*(a->input[0]-a->zeroPosition*a->input[1])+a->output[1];
	if (a->output[0]> 100){
		a->output[0]=100;
	}
	if (a->output[0]< -100){
		a->output[0]=-100;
	}
	if (a->output[0]>=0){
		a->rotDirection=1;
	}
	else{
		a->rotDirection=0;
	}
	return fabs(a->output[0]);


}
void resetSpeedControl(struct controlData *a){
	a->input[1]=0;
	a->input[0]=0;
	a->error=0;
	a->output[1]=0;
	a->output[0]=0;
	a->rotDirection=1;

	return ;
}

double applyPosControl( struct controlData *a){
	a->input[1]=a->input[0];

	if (a->pos_ref > a->maxy)
		a->error=a->maxy - a->pos_c;
	else if (a->pos_ref < a->miny)
		a->error=a->miny - a->pos_c;

	a->input[0]=a->error*0.5+a->error*pow(2.58,a->error);

	a->output[1]=a->output[0];
	a->output[0]=a->K*(a->input[0]-a->zeroPosition*a->input[1]);
	if (a->error>=0){
		a->rotDirection=1;
	}
	else{
		a->rotDirection=0;
	}

	if (a->output[0] > 100)
		a->output[0] = 100;
	else if (a->output[0] < -100)
		a->output[0] = -100;

	if ((a->output[0] - a->output[1]) > a->deltaY)
		a->output[0] = a->output[1] + a->deltaY;
	else if ((a->output[1] - a->output[0]) > a->deltaY)
		a->output[0] = a->output[1] - a->deltaY;

	return fabs(a->output[0]);
}

void resetPosControl( struct controlData *a){
	a->input[1]=0;
	a->input[0]=0;
	a->error=0;
	a->output[1]=0;
	a->output[0]=0;
	a->rotDirection=1;
	return;
}

void setMotorControlConstraints( struct controlData *a, float new_maxy, float new_miny,
		float new_deltaY, float new_Umin, float new_Umax,
		float new_delta_U, double new_alpha, int new_constraint)
{
    a->maxy = new_maxy;
    a->miny = new_miny;
    a->deltaY = new_deltaY;
    a->minu = new_Umin;
    a->maxu = new_Umax;
    a->deltaU = new_delta_U;
    a->alpha = new_alpha;
    a->constraints = new_constraint;
}
