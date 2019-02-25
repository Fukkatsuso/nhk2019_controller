/*
 * init2.h
 *
 *  Created on: 2018/09
 *      Author: mutsuro
 */
#ifndef INIT2_H_
#define INIT2_H_

#include "mbed.h"
#include "pspad.h"

#define CYCLE 5000 //動作周期(us)


/*----------------------
 -----機能選択するピン-----
 ----------------------*/
extern Pspad ps;



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

void psCommand();

#endif /* INIT2_H_ */

