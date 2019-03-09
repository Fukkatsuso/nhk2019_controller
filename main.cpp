/*
 * PSコンで速度送る場合のみ
 * speed, direction
 *
 * ◯	:最初
 * ×	:SandDune
 * △	:Tussock1
 * □	:Start2
 *
 * Start2 		& ↑	:StartClimb1
 * StartClimb1 	& ↓	:StartClimb2
 * StartClimb2 	& ←	:MountainArea
 */

#include "mbed.h"
#include "Pins.h"
#include "functions.h"
#include "Walk/CANs/CANSender.h"
#include "Walk/MRMode.h"

//#define SPEED_MAX 80.0f//90.0f


CANSender can_sender(&can);
MRMode MRmode(MRMode::GobiArea, &can_sender);

void mode_command();
void walk_command(float *speed, float *direction, float speed_max);


int main(){
	float speed = 0;
	float direction = 0;
	float speed_max = 90.0f;
	can.frequency(1000000);
	wait_ms(300); //全ての基板の電源が入るまで待つ
	pc.baud(921600);


	while(1){
		AdjustCycle(5000);
		psCommand();
		walk_command(&speed, &direction, speed_max);

		mode_command();

		can_sender.send(CANID_generate(CANID::FromMaster, CANID::ToSlaveAll, CANID::Speed), speed);
		can_sender.send(CANID_generate(CANID::FromMaster, CANID::ToSlaveAll, CANID::Direction), direction);

		//DEBUG
		pc.printf("mode:%2d  ", MRmode.get_now());
		pc.printf("Rx:%3d  Ry:%3d  ", Rx, Ry);
		pc.printf("speed:%4.3f  dir:%1.3f  ", speed, direction);
		pc.printf("\r\n");
	}
}


void mode_command(){
	//課題突入
	if		(ps.BUTTON.BIT.MARU) 	MRmode.set_initial();
	else if (ps.BUTTON.BIT.BATU) 	MRmode.set(MRMode::SandDune);
	else if (ps.BUTTON.BIT.SANKAKU) MRmode.set(MRMode::Tussock1);
	else if (ps.BUTTON.BIT.SIKAKU)	MRmode.set(MRMode::Start2);
	//登山中コマンド
	if(MRmode.get_now()==MRMode::Start2){
		if(ps.BUTTON.BIT.UP)MRmode.set(MRMode::StartClimb1);
	}
	else if(MRmode.get_now()==MRMode::StartClimb1){
		if(ps.BUTTON.BIT.DOWN)MRmode.set(MRMode::StartClimb2);
	}
	else if(MRmode.get_now()==MRMode::StartClimb2){
		if(ps.BUTTON.BIT.LEFT)MRmode.set(MRMode::MountainArea);
	}
	//送信
	can_sender.send(CANID_generate(CANID::FromMaster, CANID::ToSlaveAll, CANID::Area), MRmode.get_now());

	//			   FR:leg_up&0x1			 RR:leg_up&0x2			   FL:leg_up&0x4			 RL:leg_up&0x8
	int leg_up = ((ps.BUTTON.BIT.R1)<<0) + ((ps.BUTTON.BIT.R2)<<1) + ((ps.BUTTON.BIT.L1)<<2) + ((ps.BUTTON.BIT.L2)<<3);
	can_sender.send(CANID_generate(CANID::FromMaster, CANID::ToSlaveAll, CANID::LegUp), leg_up);
	//	if(MRmode.get_now()==MRMode::SandDune && ps.BUTTON.BIT.SIKAKU)leg_up = 0xf;
}

void walk_command(float *speed, float *direction, float speed_max){
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
	*speed = (Ry>=0? 1.0:-1.0)*(sqrt2(Rx, Ry)/(float)ANALOG_MAX) * speed_max;
	*direction = theta;
}
