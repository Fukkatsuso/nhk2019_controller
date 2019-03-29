/*
 * CANNavigation.h
 *
 *  Created on: 2019/03/28
 *      Author: mutsuro
 */

#ifndef WALK_CANS_CANNAVIGATION_H_
#define WALK_CANS_CANNAVIGATION_H_

#include "mbed.h"


union rcv_t{
	unsigned char byte[3];
	struct{
		signed short angle; //目標相対角[degree], 左回り正
		unsigned char status; //state_tの内容
	};
};

union send_t{
	unsigned char byte[2];
	signed short dist; //最初からの累計移動距離[mm]
};


class CANNavigation
{
public:
	CANNavigation(CAN *can);
	void send_dist(float dist);
	void receive(CANMessage msg);

	short get_status();
	float get_angle();

private:
	CAN *can;
	union rcv_t navi;

	enum state_t{
		Stop,
		Walk,
		SandDune,
		Tussock,
		MountainArea,
		UukhaiZone
	};
};


#endif /* WALK_CANS_CANNAVIGATION_H_ */
