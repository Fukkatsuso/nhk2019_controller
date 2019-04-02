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

	semaphore.rcv_t = true;
}


extern Serial pc;
void CANNavigation::send(short dist, unsigned char kouden_front, unsigned char kouden_rear)
{
	pc.printf("send[%5d][%1d][%1d]  ", dist, kouden_front, kouden_rear);
	union send_t data = {};
	data.dist = (signed short)dist;
	data.PhotoelectricSensor.Front = kouden_front;
	data.PhotoelectricSensor.Rear = kouden_rear;
	unsigned int can_id = CANID::generate(CANID::FromController, CANID::ToMaster, CANID::MovePosition);
	CANMessage msg(can_id, (const char*)data.byte, 3);
	if(can->write(msg));
	else if(can->write(msg));
	else if(can->write(msg));
	else pc.printf("send_error[%d][%1d][%1d]  ", data.dist, (int)data.PhotoelectricSensor.Front, (int)data.PhotoelectricSensor.Rear);
}


void CANNavigation::receive(CANMessage msg)
{
	semaphore.rcv_t = false;
	for(int i=0; i<3; i++)navi.byte[i] = msg.data[i];
	semaphore.rcv_t = true;
//	pc.printf("rcv[%d][%d] ", navi.angle, navi.status);
}


short CANNavigation::get_status()
{
	while(!semaphore.rcv_t);
	return (short)navi.status;
}


//navi.angleはdegreeなのでradに変換
//右回り正にしたいのでマイナスかける
short CANNavigation::get_angle()
{
	while(!semaphore.rcv_t);
	return navi.angle;//-(float)navi.angle * M_PI / 180.0;
}
