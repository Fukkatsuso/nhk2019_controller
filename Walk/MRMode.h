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


class MRMode
{
public:
	enum Area{
		WaitGobiUrtuu = 0,//待機
		GetGerege,//ゲルゲ受け取り検知
		PrepareWalking,
		Start1,//歩行開始
		GobiArea,//直進
		SandDune,//段差
		ReadyForTussock,//ここに何か入れるべき
		Tussock1,//紐1
		Tussock2,//紐2
		Finish1,//到着
		WaitMountainUrtuu,//待機
		GetSign,//非接触の合図
		Start2,//歩行開始
		MountainArea,//登山
		UukhaiZone,//ウーハイゾーン
		Uukhai,//ウーハイ
		Finish2,//終了
		Area_end,
	};
	enum Reference{
		Initial,
		Prev,
		Now,
		Next,
		Reference_end
	};

	MRMode(enum Area area_initial, CANSender *can_sender);
	void send();
	int get_now();

private:
	CANSender *can_sender;
	Area now;
};


#endif /* WALK_MRMODE_H_ */
