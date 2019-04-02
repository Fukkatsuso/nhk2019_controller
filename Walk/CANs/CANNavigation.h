/*
 * CANNavigation.h
 *
 *  Created on: 2019/03/28
 *      Author: mutsuro
 */

#ifndef WALK_CANS_CANNAVIGATION_H_
#define WALK_CANS_CANNAVIGATION_H_

#include "mbed.h"


//Master->Controller : angle, status 					: 0x013
//Master<-Controller : dist, kouden_front, kouden_rear	: 0x102

union rcv_t{
	unsigned char byte[3];
	struct{
		signed short angle; //目標相対角[degree], 左回り正
		unsigned char status; //state_tの内容
	};
};

union send_t{
	unsigned char byte[3];
	struct{
		signed short dist; //最初からの累計移動距離[mm]
		struct{ //光電センサ:onで1
			unsigned char Front : 1;
			unsigned char Rear  : 1;
		}PhotoelectricSensor;
	};
};


class CANNavigation
{
public:
	CANNavigation(CAN *can);
	void send(short dist=0, unsigned char kouden_front=0, unsigned char kouden_rear=0);
	void receive(CANMessage msg);

	short get_status();
	short get_angle();

	enum state_t{
		Stop,
		Walk,
		SandDune,
		Tussock,
		MountainArea,
		UukhaiZone
	};

private:
	CAN *can;
	union rcv_t navi;

	struct{
		bool rcv_t;
	}semaphore;
};


#endif /* WALK_CANS_CANNAVIGATION_H_ */
