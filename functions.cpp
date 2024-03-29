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


float trapezoidal_control(float now, float initial, float target, float roop_period, float time_required){
	if(time_required==0)return target;
	float next = now + (target - initial) * roop_period / time_required;
	if(initial < target)return limit(next, target, initial);
	else return limit(next, initial, target);
}


unsigned int counter_update(unsigned int counter, unsigned char flag)
{
	if(flag)counter++;
	else if(counter > 0)counter--;
	return counter;
}


//歩行角度に補正かける
int adjust_walk_direction(int direction){
	//2PI以上の角度は切り落とす
	int half = 180;
	while(abs(direction) > half){
		if(direction > half)direction -= half*2;
		else if(direction < -half)direction += half*2;
	}

	//曲進角度を2倍に増幅:制限付き
	//したかったけど考えるの面倒でやっぱりやめる

	return direction;
}


//main用
void stick_zero(int *stick, int margin){
	if(*stick > margin)*stick -= margin;
	else if(*stick < -margin)*stick += margin;
	else *stick = 0;
}
