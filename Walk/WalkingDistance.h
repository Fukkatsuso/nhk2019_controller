/*
 * WalkingDistance.h
 *
 *  Created on: 2019/03/26
 *      Author: mutsuro
 */

#ifndef WALK_WALKINGDISTANCE_H_
#define WALK_WALKINGDISTANCE_H_

#include "mbed.h"


class WalkingDistance
{
public:
	WalkingDistance();
	void integrate_move_fr(float move_fr);
	void integrate_move_fl(float move_fl);
	void integrate_move_rr(float move_rr);
	void integrate_move_rl(float move_rl);
	void calc_position(float duty);

	void reset_position(float x, float y, float dir);
	void reset_move_right();
	void reset_move_left();

	float get_x();
	float get_y();
	float get_direction();

protected:
	float calc_dif_direction(float dif_right, float dif_left);

private:
	struct{
		//微小歩行量[mm]
		//CAN割り込み受信時に積算
		float fr;
		float fl;
		float rr;
		float rl;
	}move;

	struct{
		//総歩行距離[mm]
		float x;
		float y;
		//機体が向いている方向[rad]
		float direction;
	}position;

	//読み書きokならばtrueにする
	struct{
		bool fr;
		bool fl;
		bool rr;
		bool rl;
	}semaphore;
};

/*
 * 10m = 10000mm
 *
 * type = WalkDist@@
 * 正負部:1桁
 * 整数部:2桁
 * 小数部:5桁
 */


#endif /* WALK_WALKINGDISTANCE_H_ */
