/*
 * Pins.h
 *
 *  Created on: 2019/03/02
 *      Author: mutsuro
 */

#ifndef PINS_H_
#define PINS_H_

#include "mbed.h"
#include "PhotoelectricSensor.h"
#include "pspad.h"

//#define CYCLE 5000 //動作周期(us)
#define ANALOG_MAX 128
#define ANALOG_MARGIN 50


/*----------------------
 ----mbed本体上のピン-----
 ----------------------*/
extern Serial pc;
extern DigitalOut led0;
extern DigitalOut led1;
extern DigitalOut led2;
extern DigitalOut led3;
extern Timer AdCycle; //AdjustCycleで使うタイマ


/*----------------------
 -----機能選択するピン-----
 ----------------------*/
extern CAN can;
extern PhotoelectricSensor kouden_Mt;
extern Pspad ps;


/************************
 * 		駆動系/その他		*
 ************************/
//extern PwmOut sv_gerege;
extern DigitalIn sw_gerege;
extern DigitalIn sw_stop;
extern PhotoelectricSensor kouden_urtuu2_right;
extern PhotoelectricSensor kouden_urtuu2_left;
extern PhotoelectricSensor kouden_dune_detect;
extern DigitalOut cyl_gerege;



/*----------------------
 ---------関数---------
 ----------------------*/
void AdjustCycle(int t_us);


/*----------------------
  	  	psCommand
  ---------------------*/
extern int up;
extern int down;
extern int right;
extern int left;

extern int Rx;
extern int Ry;
extern int Lx;
extern int Ly;

extern int R1;
extern int R2;
extern int L1;
extern int L2;

extern void psCommand();
extern float lim_stick(int *rx, int *ry);


#endif /* PINS_H_ */
