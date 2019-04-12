/*
 * MRMode.cpp
 *
 *  Created on: 2019/03/03
 *      Author: mutsuro
 */


#include "MRMode.h"
#include "Walk/CANs/CANNavigation.h"
#include "functions.h"

extern Serial pc;

const float MRMode::params[MRMode::Area_end][MRMode::Params_end] =
{
		//Speed_max,	//Time_speed_change
//Walk
		{0,				0},		//WaitGobiUrtuu
		{0,				0},		//GetGerege
		{0,				0},		//PrepareWalking
		{50,			0},		//Start1 //GobiAreaのスピード初期値
		{
				350,
//				400,
				2},	//GobiArea
//SandDune
		{133,			0},		//SandDuneFront
		{133,			0},		//SandDuneRear
//Walk
		{350/*240*/,	2/*1*/},		//ReadyForTussock
//Tussock
		{200,			1},		//Tussock
		{200,			1},		//Finish1
//MountainArea
		{0,				0},		//WaitMountainUrtuu
		{0,				0},		//GetSign
		{240,			1.5},	//Start2
		{270,			0},		//StartClimb1
		{270,			0},		//StartClimb2
		{270,			0},		//MountainArea
		{270,			0},		//UukhaiZone
//UukhaiZone
		{270,			0},		//Uukhai
		{50,			0}		//Finish2
};


MRMode::MRMode(enum Area area_initial, CANSender *can_sender)
{
	area.semaphore = false;
	area.now = area_initial;
	area.prev = area_initial;
	area.initial = area_initial;
	area.semaphore = true;
	this->can_sender = can_sender;

	direction = 0;
	speed = 0;

	for(int i=0; i<MRMode::Area_end; i++){
		dists[i] = 0;
	}

	timer_switching.reset();
	timer_switching.start();
}


void MRMode::set_sensors(DigitalIn *sw_gerege, DigitalIn *sw_stop,
		PhotoelectricSensor *kouden_urtuu2, PhotoelectricSensor *kouden_dune_detect)
{
	this->sw_gerege = sw_gerege;
	this->sw_stop = sw_stop;
	gerege.now = gerege.prev = sw_gerege->read();
	this->kouden_urtuu2 = kouden_urtuu2;
	this->kouden_dune_detect = kouden_dune_detect;
}


void MRMode::set_initial()
{
	area.semaphore = false;
	area.now = area.initial;
	area.semaphore = true;
}


void MRMode::set(enum Area area)
{
	this->area.semaphore = false;
	//変更のみ受け付ける
	if(area!=this->area.now){
		timer_switching.reset();
		timer_switching.start();
		this->area.prev = this->area.now;
		this->area.now = area;
	}
	this->area.semaphore = true;
}


int MRMode::get_now()
{
	while(!area.semaphore);
	return area.now;
}


float MRMode::get_speed()
{
	return speed;
}


short MRMode::get_direction()
{
	return direction;
}


float MRMode::get_time()
{
	return timer_switching.read();
}


float MRMode::get_speed_max()
{
	while(!area.semaphore);
	return params[area.now][Speed_max];
}


short MRMode::get_dist_in_area()
{
	while(!area.semaphore);
	return dists[area.now] - dists[area.prev];
}

unsigned char MRMode::get_leg_up()
{
	return leg_up;
}


//CANNavigationの指令, 総歩行量, 前後の光電センサのカウンタ
void MRMode::plan(short navi_status, short navi_angle, short dist, unsigned int kouden_sd_f, unsigned int kouden_sd_r)
{
	sensor_update();
	dists_update(dist);

	//can_naviからのstatusに少し修正
	short navi_state = plan_mode(navi_status, kouden_sd_f, kouden_sd_r);

	//速度の計算
	plan_velocity(navi_state, navi_angle);
//	pc.printf("ang[%f]  ", navi_angle); //謎のprif解決(メモリリーク?)
}


void MRMode::sensor_update()
{
	gerege.prev = gerege.now;
	gerege.now = sw_gerege->read();

	kouden_urtuu2->sensing();
	kouden_dune_detect->sensing();
}


void MRMode::dists_update(short dist)
{
	while(!area.semaphore);
	dists[area.now] = dist;
}


void MRMode::plan_velocity(short navi_status, short navi_angle)
{
	direction = navi_angle;

	switch(navi_status){
	case CANNavigation::Stop:
		direction = 0;
		speed = 0;
		break;

	case CANNavigation::Walk:
		while(!area.semaphore);
		speed = trapezoidal_control(speed, params[area.prev][Speed_max], params[area.now][Speed_max], 0.005, params[area.now][Time_change_speed]);
		break;

	case CANNavigation::SandDune:
		while(!area.semaphore);
		speed = params[area.now][Speed_max];
		break;

	case CANNavigation::Tussock:
		while(!area.semaphore);
		speed =  trapezoidal_control(speed, params[area.prev][Speed_max], params[area.now][Speed_max], 0.005, params[area.now][Time_change_speed]);
		break;

	case CANNavigation::MountainArea:
		while(!area.semaphore);
		speed = params[area.now][Speed_max];
		break;

	case CANNavigation::UukhaiZone:
		speed = 0;
		break;
	}

	if(cos(((float)direction)*M_PI/180.0)<0)speed = -fabs(speed);
}


//CANNavigationのモード指示, 前後の光電センサのカウント
short MRMode::plan_mode(short navi_status, unsigned int kouden_sd_f, unsigned int kouden_sd_r)
{
	leg_up = 0;
	MRMode::Area area_now = (Area)get_now();

	switch(area_now){
	case WaitGobiUrtuu:
		if(navi_status==CANNavigation::SandDune) set(MRMode::SandDuneFront);
		else if(navi_status==CANNavigation::Tussock) set(MRMode::Tussock);
		else if(navi_status==CANNavigation::MountainArea) set(MRMode::WaitMountainUrtuu);
		if(gerege.now && !gerege.prev) set(MRMode::GetGerege);
		navi_status = CANNavigation::Stop;
		break;

	case GetGerege:
		navi_status = CANNavigation::Stop;
		if(timer_switching.read()>1) set(MRMode::PrepareWalking); //受取待機マージン
		break;

	case PrepareWalking:
		navi_status = CANNavigation::Stop;
		if(timer_switching.read()>1) set(MRMode::Start1); //姿勢調整待機
		break;

	case Start1: //CANNavigation::Walk
		set(MRMode::GobiArea);
		break;

	case GobiArea:
		if(navi_status==CANNavigation::SandDune)flag.sanddune = true;
		if(flag.sanddune && kouden_dune_detect->get_counter() > 100)
			set(MRMode::SandDuneFront);
		break;

	case SandDuneFront:
		//指令がFrontから送られてきたら自動的に次へ進む
		//若干待機してから変更した方がよさそう
		break;

	case SandDuneRear:
//		if(navi_status!=CANNavigation::SandDune){ //SandDuneクリア(?)
//			if(kouden_sd_r < 700) set(MRMode::ReadyForTussock);
//			else set(MRMode::SandDuneRear);
//		}
		//Rearから変更申請して次のモードへ移行
		break;

	case ReadyForTussock:
//		if(kouden_sd_r > 700) set(MRMode::SandDuneRear); //一応。謎のバグ対策(4/1)
//		else
			if(navi_status==CANNavigation::Tussock) set(MRMode::Tussock);
		break;

	case Tussock:
//		if(timer_switching.read()>1.5)//少し待機
			leg_up = 0xf; //最初は前脚のみ、少し待って後脚も上げるとかに変更?
		if(navi_status==CANNavigation::Stop) set(MRMode::Finish1);
		if(navi_status==CANNavigation::MountainArea){
			navi_status = CANNavigation::Stop;
			set(MRMode::Finish1);
		}
		if(sw_stop->read()){
			navi_status = CANNavigation::Stop;
			set(MRMode::Finish1);
		}
		break;

	case Finish1:
		leg_up = 0xf; //一応
		if(navi_status==CANNavigation::MountainArea) set(MRMode::WaitMountainUrtuu);
		break;

	case WaitMountainUrtuu:
		navi_status = CANNavigation::Stop;
		if(kouden_urtuu2->get_ontime() > 0.3) set(MRMode::GetSign);
		break;

	case GetSign://すでに合図を受け取っている
		navi_status = CANNavigation::Stop;
		if(timer_switching.read() > 1) //姿勢変更待機
			set(MRMode::Start2);
		break;

	case Start2:
		navi_status = CANNavigation::Walk;
		if(get_dist_in_area() > 490.0*1.2) set(MRMode::StartClimb1);
		break;

	case StartClimb1:
		//足の前後の中止間の距離:520mm
		//Urtuu2中心から坂の初めまでの距離:750mm
		//坂の始めまでに歩くべき距離:750-520/2 = 490mm
		//坂道の長さ:1600mm
		navi_status = CANNavigation::MountainArea;
		if(get_dist_in_area() > 550) set(MRMode::MountainArea); //後ろ足専用のモーションはすっ飛ばす
		break;

	case StartClimb2:
		if(get_dist_in_area() > 260) set(MRMode::MountainArea); //実質使ってない
		break;

	case MountainArea:
		if(navi_status==CANNavigation::UukhaiZone) set(MRMode::UukhaiZone);
		else if(navi_status==CANNavigation::Stop) set(MRMode::UukhaiZone);
		break;

	case UukhaiZone:
		if(timer_switching.read() > 3) //初期位置戻して落ち着くまで待機  //将来的にはSlaveから完全停止フラグを受け取る
			set(MRMode::Uukhai);
		break;

	case Uukhai:
		if(timer_switching.read() > 1) //main()内でゲルゲ掲げるプログラム
			set(MRMode::Finish2);
		break;

	case Finish2:
		navi_status = CANNavigation::Stop;
		break;
	}

	return navi_status;
}
