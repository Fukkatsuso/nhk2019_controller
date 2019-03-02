/*
 * functions.cpp
 *
 *  Created on: 2018/11/16
 *      Author: mutsuro
 */

#include "functions.h"

int limit(int value, int max, int min){
	if(value > max)return max;
	else if(value < min)return min;
	else return value;
}

double limit(double value, double max, double min){
	if(value > max)return max;
	else if(value < min)return min;
	else return value;
}

float sqrt2(float a, float b){
	return sqrt(a*a + b*b);
}

float cos_formula(float A1, float A2, float B){
	return ((A1)*(A1) + (A2)*(A2) - B*B) / (2 * (A1) * (A2));
}


//main用
void stick_zero(int *stick, int margin){
	if(*stick > margin)*stick -= margin;
	else if(*stick < -margin)*stick += margin;
	else *stick = 0;
}


void up_gerege(float *duty, float *duty_prev){
	*duty = *duty_prev +
			limit(((float)SERVO_GEREGE_UP-(float)SERVO_GEREGE_INIT),
					fabs(SERVO_GEREGE_UP-SERVO_GEREGE_INIT)*(((float)ROOP_PERIOD/1000000.0)/TIME_GEREGE_UP),
					-fabs(SERVO_GEREGE_UP-SERVO_GEREGE_INIT)*(((float)ROOP_PERIOD/1000000.0)/TIME_GEREGE_UP));
	*duty = limit(*duty, (float)SERVO_GEREGE_INIT, (float)SERVO_GEREGE_UP);
	*duty_prev = *duty;
}
