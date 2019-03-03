/*
 * PSコンで速度送る場合のみ
 * speed, direction
 */

#include "mbed.h"
#include "Pins.h"
#include "functions.h"
#include "Walk/CANs/CANSender.h"
#include "Walk/MRMode.h"

#define SPEED_MAX 100.0f


CANSender can_sender(&can);
MRMode MRmode(MRMode::GobiArea, &can_sender);

void walk_command(float *speed, float *direction);


int main(){
	float speed = 0;
	float direction = 0;
	can.frequency(1000000);
	wait_ms(300); //全ての基板の電源が入るまで待つ
	pc.baud(921600);


	while(1){
		AdjustCycle(5000);
		psCommand();
		walk_command(&speed, &direction);

//		MRmode.send();
		can_sender.send(CANID_generate(CANID::FromMaster, CANID::ToSlaveAll, CANID::Area), MRmode.get_now());
		can_sender.send(CANID_generate(CANID::FromMaster, CANID::ToSlaveAll, CANID::Speed), speed);
		can_sender.send(CANID_generate(CANID::FromMaster, CANID::ToSlaveAll, CANID::Direction), direction);

		//DEBUG
		pc.printf("mode:%2d  ", MRmode.get_now());
		pc.printf("Rx:%3d  Ry:%3d  ", Rx, Ry);
		pc.printf("speed:%4.3f  dir:%1.3f  ", speed, direction);
		pc.printf("\r\n");
	}
}


void walk_command(float *speed, float *direction){
	float theta;
	if((-ANALOG_MARGIN<Ry && Ry<ANALOG_MARGIN) && (-ANALOG_MARGIN<Rx && Rx<ANALOG_MARGIN)){
		theta = 0;
		Ry = 0;
		Rx = 0;
	}
	else theta = atan2(Rx, Ry);
	if(sqrt2(Rx, Ry)>ANALOG_MAX){
		Rx = ANALOG_MAX*sin(theta);
		Ry = ANALOG_MAX*cos(theta);
	}
	*speed = (Ry>=0? 1.0:-1.0)*(sqrt2(Rx, Ry)/(float)ANALOG_MAX) * SPEED_MAX;
	*direction = theta;
}
