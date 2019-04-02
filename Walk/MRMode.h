/*
 * MRMode.h
 *
 *  Created on: 2019/02/16
 *      Author: mutsuro
 */

#ifndef WALK_MRMODE_H_
#define WALK_MRMODE_H_

#include "mbed.h"
//#include "FieldMap.h"
#include "Walk/CANs/CANSender.h"
#include "PhotoelectricSensor.h"


struct Sensor{
	short now;
	short prev;
};

class MRMode
{
public:
	enum Area{
		WaitGobiUrtuu = 0,	//待機
		GetGerege,			//ゲルゲ受け取り検知
		PrepareWalking,
		Start1,				//歩行開始
		GobiArea,			//直進
		SandDuneFront,		//段差
		SandDuneRear,
		ReadyForTussock,	//ここに何か入れるべき
		Tussock,			//紐
		Finish1,			//到着
		WaitMountainUrtuu,	//待機
		GetSign,			//非接触の合図
		Start2,				//歩行開始
		StartClimb1,		//Front登山開始
		StartClimb2,		//Rear
		MountainArea,		//登山
		UukhaiZone,			//ウーハイゾーン
		Uukhai,				//ウーハイ
		Finish2,			//終了
		Area_end,
	};

	enum Params{
		Speed_max,
		Time_change_speed,
		Params_end
	};

	MRMode(enum Area area_initial, CANSender *can_sender);
	void set_sensors(DigitalIn *sw_gerege, PhotoelectricSensor *kouden_urtuu2);
	void set_initial();
	void set(enum Area area);
	void send();

	int get_now();
	float get_speed();
	float get_direction();
	float get_time();
	float get_speed_max();
	short get_dist_in_area();

	void plan(short navi_status, float navi_angle, short dist, unsigned int kouden_sd_f, unsigned int kouden_sd_r);

//protected:
	short plan_mode(short navi_status, unsigned int kouden_sd_f, unsigned int kouden_sd_r);
	void plan_velocity(short navi_status, float navi_angle=0);
	void sensor_update();
	void dists_update(short dist);

private:
	Timer timer_switching;
	CANSender *can_sender;
	Area now;
	Area prev;
	Area initial;

	DigitalIn *sw_gerege;
	PhotoelectricSensor *kouden_urtuu2;

	Sensor gerege;

	float speed;
	float direction;

	short dists[MRMode::Area_end]; //モード切替までに歩いた距離

	static const float params[MRMode::Area_end][MRMode::Params_end];
};


#endif /* WALK_MRMODE_H_ */
