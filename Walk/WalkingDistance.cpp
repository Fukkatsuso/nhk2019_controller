/*
 * WalkingDistance.cpp
 *
 *  Created on: 2019/03/26
 *      Author: mutsuro
 */

#include "WalkingDistance.h"

//半径:225mm
#define RADIUS 225.0


WalkingDistance::WalkingDistance()
{
	semaphore.fr = true;
	semaphore.fl = true;
	semaphore.rr = true;
	semaphore.rl = true;
	reset_move_right();
	reset_move_left();
	reset_position(0, 0, 0);
}


void WalkingDistance::integrate_move_fr(float move_fr)
{
	semaphore.fr = false;
	move.fr += move_fr;
	semaphore.fr = true;
}


void WalkingDistance::integrate_move_fl(float move_fl)
{
	semaphore.fl = false;
	move.fl += move_fl;
	semaphore.fl = true;
}


void WalkingDistance::integrate_move_rr(float move_rr)
{
	semaphore.rr = false;
	move.rr += move_rr;
	semaphore.rr = true;
}


void WalkingDistance::integrate_move_rl(float move_rl)
{
	semaphore.rl = false;
	move.rl += move_rl;
	semaphore.rl = true;
}


//mainループ内で毎ループ1回実行
void WalkingDistance::calc_position(float duty)
{
	while(!(semaphore.fr && semaphore.fl));
	float move_right = -(move.fr + move.rr) / (duty * 2.0);
	reset_move_right();

	while(!(semaphore.rr && semaphore.rl));
	float move_left = -(move.fl + move.rl) / (duty * 2.0);
	reset_move_left();

	float move_center = (move_right + move_left) / 2.0;

	position.direction += calc_dif_direction(move_right, move_left);

	position.x += move_center * sin(position.direction);
	position.y += move_center * cos(position.direction);
}


void WalkingDistance::reset_position(float x, float y, float dir)
{
	position.x = x;
	position.y = y;
	position.direction = dir;
}


void WalkingDistance::reset_move_right()
{
	while(!semaphore.fr);move.fr = 0;
	while(!semaphore.fl);move.fl = 0;
}


void WalkingDistance::reset_move_left()
{
	while(!semaphore.rr);move.rr = 0;
	while(!semaphore.rl);move.rl = 0;
}


float WalkingDistance::calc_dif_direction(float move_right, float move_left)
{
	float angle_right = atan2(move_right, RADIUS);
	float angle_left = atan2(move_left, RADIUS);
	return angle_left - angle_right;
}


float WalkingDistance::get_x()
{
	return position.x;
}


float WalkingDistance::get_y()
{
	return position.y;
}


float WalkingDistance::get_direction()
{
	return position.direction * 180.0/M_PI;//[degree]
}
