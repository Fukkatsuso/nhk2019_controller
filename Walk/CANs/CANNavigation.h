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

#define BYTE_RCV_T 3
union rcv_t{
	unsigned char byte[BYTE_RCV_T];
	struct{
		signed short angle; //目標相対角[degree], 左回り正
		unsigned char status; //state_tの内容
	};
};

#define BYTE_SEND_T 3
union send_t{
	unsigned char byte[BYTE_SEND_T];
	struct{
		signed short dist; //最初からの累計移動距離[mm]
		union{ //光電センサ:onで1
			unsigned char byte[1];
			struct{
				unsigned char Front : 1;
				unsigned char Rear  : 1;
			};
		}PhotoelectricSensor;
	};
};


class CANNavigation
{
public:
	CANNavigation(CAN *can);
	void send(short dist, unsigned char kouden_front, unsigned char kouden_rear);
	void receive(CANMessage msg);

	unsigned char get_status();
	signed short get_angle();

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
	struct{
		union rcv_t data;
		bool semaphore;
	}navi;
};


#endif /* WALK_CANS_CANNAVIGATION_H_ */
