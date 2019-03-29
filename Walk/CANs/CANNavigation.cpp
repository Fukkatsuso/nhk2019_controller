/*
 * CANNavigation.cpp
 *
 *  Created on: 2019/03/29
 *      Author: mutsuro
 */


#include "CANNavigation.h"
#include "CANProtocol.h"


CANNavigation::CANNavigation(CAN *can)
{
	this->can = can;
	navi.angle = 0;
	navi.status = CANNavigation::Stop;
}


void CANNavigation::send_dist(float dist)
{
	union send_t data;
	data.dist = (signed short)dist;
	unsigned int can_id = CANID::generate(CANID::FromController, CANID::ToMaster, CANID::MoveDistCentroid);

	CANMessage msg(can_id, (const char*)data.byte, 2);
	if(can->write(msg));
	else if(can->write(msg));
	else (can->write(msg));
}


void CANNavigation::receive(CANMessage msg)
{
	for(int i=0; i<3; i++)navi.byte[i] = msg.data[i];
}


short CANNavigation::get_status()
{
	return (short)navi.status;
}


//navi.angleはdegreeなのでradに変換
//右回り正にしたいのでマイナスかける
float CANNavigation::get_angle()
{
	return -(float)navi.angle * M_PI / 180.0;
}
