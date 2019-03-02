/*
 * functions.h
 *
 *  Created on: 2018/11/09
 *      Author: mutsuro
 *
 *  汎用的な関数を定義
 */

#ifndef FUNCTIONS_H_
#define FUNCTIONS_H_

#include "mbed.h"

#define ROOP_PERIOD 5000//whileループ周期[us]

#define SERVO_GEREGE_FREE 50
#define SERVO_GEREGE_INIT 2150
#define SERVO_GEREGE_UP 1000
#define TIME_GEREGE_UP 1.0f//持ち上げに要する時間[s]


//リミット:int
int limit(int value, int max, int min);

//リミット:double
double limit(double value, double max, double min);

//二乗和平方根
float sqrt2(float a, float b);

//余弦定理
float cos_formula(float A1, float A2, float B);


//main用
void stick_zero(int *stick, int margin);
void up_gerege(float *duty, float *duty_prev);

#endif /* FUNCTIONS_H_ */
