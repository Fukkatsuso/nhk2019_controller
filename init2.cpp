/*
 * init2.cpp
 *
 *  Created on: 2018/09
 *      Author: mutsuro
 */

#include "init2.h"

/*----------------------
 -----機能選択するピン-----
 ----------------------*/
Pspad ps(p17, p18, p19, p20);



/*----------------------
 ----mbed本体上のピン-----
 ----------------------*/
Serial pc(USBTX, USBRX);
DigitalOut led0(LED1);
DigitalOut led1(LED2);
DigitalOut led2(LED3);
DigitalOut led3(LED4);
Timer AdCycle;



/*
 * 関数名	AdjustCycle
 *
 * 用途		マイコンの動作周期を調整する
 *
 * 引数		int t_us:目的の動作周期(us)
 *
 * 戻り値		なし
 *
 * 備考		関数実行時、前の実行時からt_us経っていない場合、t_us経つまで待つ
 * 			すでにt_us経っている場合、led3を点灯する
 */
void AdjustCycle(int t_us){
    if(AdCycle.read_us() == 0) AdCycle.start();

    if(AdCycle.read_us()>t_us){
    	led3=1;
    }
    else {
    	led3=0;
    }
    while(AdCycle.read_us()<=t_us);
    AdCycle.reset();
}



/*----------------------
  	  	psCommand
  ---------------------*/
int up;
int down;
int right;
int left;

int Rx;
int Ry;
int Lx;
int Ly;

int R1;
int R2;
int L1;
int L2;

void psCommand(){

}
