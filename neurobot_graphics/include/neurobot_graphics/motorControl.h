/*
 * velocityControl.h
 *
 *  Created on: Mar 4, 2024
 *      Author: juan
 */

#ifndef INC_VELOCITYCONTROL_H_
#define INC_VELOCITYCONTROL_H_

#include <math.h>

struct controlData{
	double output[2];
	double input[2];
	double K;
	double zeroPosition;
	double error;
	double pos_c;
	double pos_ref;
	double positionReference;
	double speedReference;
	double rotDirection;

	double alpha=0;
	int constraints=0;

	float maxy = 20;
	float miny = -20;
	float deltaY = 100;
	float minu=0.50;
	float maxu=0.50;
	float deltaU=0.2;

};
void initControlStructure(double K, double zeroPosition, struct controlData *a);

double applySpeedControl( struct controlData *a);
void resetSpeedControl( struct controlData *a);

double applyPosControl( struct controlData *a);
void resetPosControl( struct controlData *a);

void setMotorControlConstraints( struct controlData *a, float new_maxy, float new_miny,
		float new_deltaY, float new_Umin, float new_Umax,
		float new_delta_U, double new_alpha, int new_constraint);

#endif /* INC_VELOCITYCONTROL_H_ */
