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
	navi.data.angle = 0;
	navi.data.status = CANNavigation::Stop;
	navi.semaphore = true;
}


extern Serial pc;
void CANNavigation::send(short dist, unsigned char kouden_front, unsigned char kouden_rear)
{
	pc.printf("send[%5d][%1d][%1d]  ", dist, kouden_front, kouden_rear);
	union send_t data = {};
	data.dist = (signed short)dist;
	data.PhotoelectricSensor.Front = kouden_front;
	data.PhotoelectricSensor.Rear = kouden_rear;
	unsigned int can_id = CANID::generate(CANID::FromController, CANID::ToMaster, CANID::MRInfo);
	CANMessage msg(can_id, (const char*)data.byte, BYTE_SEND_T);
	if(can->write(msg));
	else if(can->write(msg));
	else if(can->write(msg));
	else pc.printf("send_error[%d][%1d][%1d]  ", data.dist, data.PhotoelectricSensor.Front, data.PhotoelectricSensor.Rear);
}


void CANNavigation::receive(CANMessage msg)
{
	navi.semaphore = false;
	for(int i=0; i<BYTE_RCV_T; i++)navi.data.byte[i] = msg.data[i];
	navi.semaphore = true;
}


unsigned char CANNavigation::get_status()
{
	while(!navi.semaphore);
	return navi.data.status;
}


//angleはdegree単位
//右回り正にしたいので使うときはマイナスかける
signed short CANNavigation::get_angle()
{
	while(!navi.semaphore);
	return navi.data.angle;//-(float)navi.angle * M_PI / 180.0;
}
