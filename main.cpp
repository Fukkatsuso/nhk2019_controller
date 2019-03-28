/*
 * PSコンで速度送る場合のみ
 * speed, direction
 *
 * ◯	:最初
 * ×	:SandDuneFront
 * △	:Tussock1
 * □	:Start2
 *
 * SandDuneFront & ↓ :SandDuneRear
 * SandDuneRear & ↑ :最初
 *
 * Start2 		& ↑	:StartClimb1
 * StartClimb1 	& ↓	:StartClimb2
 * StartClimb2 	& ←	:MountainArea
 */

#include "mbed.h"
#include "Pins.h"
#include "functions.h"
#include "Walk/CANs/CANSender.h"
#include "Walk/CANs/CANReceiver.h"
#include "Walk/MRMode.h"


CANMessage rcvMsg;
CANSender can_sender(&can);
CANReceiver can_receiver(&can);
MRMode MRmode(MRMode::GobiArea, &can_sender);

void CANrcv();
void mode_command();
void walk_command(float *speed, float *direction, float speed_max);

float walk_dist = 0;;


int main(){
	float speed = 0;
	float direction = 0;
	float speed_max = 90.0f;
	int switch_g_now = 0;
	int switch_g_prev = 0;

	walk_dist = 0;

	can.frequency(1000000);
	can.attach(&CANrcv, CAN::RxIrq);
	wait_ms(300); //全ての基板の電源が入るまで待つ
	pc.baud(921600);

	while(1){
		AdjustCycle(5000);
		switch_g_prev = switch_g_now;
		switch_g_now = sw_gerege.read();
		psCommand();

		if(ps.BUTTON.BIT.SELECT)walk_dist = 0;

		switch(MRmode.get_now()){
		case MRMode::GobiArea:
			speed_max = 240;
			break;
		case MRMode::SandDuneFront:
			speed_max = 100;
			break;
		case MRMode::SandDuneRear:
			speed_max = 100;
			break;
		case MRMode::Tussock:
			speed_max = 240;
			break;
		case MRMode::Start2:
			speed_max = 240;//120;
			break;
		case MRMode::StartClimb1:
			speed_max = 220;
			break;
		}

		walk_command(&speed, &direction, speed_max);

		mode_command();
		//ゲルゲ受け取りから歩き出すまで自動化
		//MRmode初期状態:WaitGobiUrtuu
//		if(switch_g_now && !switch_g_prev){
//			if(MRmode.get_now()==MRMode::WaitGobiUrtuu){
//				MRmode.set(MRMode::GetGerege);
//				can_sender.send(CANID_generate(CANID::FromController, CANID::ToSlaveAll, CANID::Area), MRMode::GetGerege);
//				wait(1);
//				MRmode.set(MRMode::PrepareWalking);
//				can_sender.send(CANID_generate(CANID::FromController, CANID::ToSlaveAll, CANID::Area), MRMode::PrepareWalking);
//				wait(1);
//				MRmode.set(MRMode::GobiArea);
//				can_sender.send(CANID_generate(CANID::FromController, CANID::ToSlaveAll, CANID::Area), MRMode::GobiArea);
//			}
//		}
//		if(MRmode.get_now()==MRMode::GobiArea){
//			speed = limit(speed + 240.0*0.005, 240.0, 0.0);
//			direction = 0;
//		}

		can_sender.send(CANID_generate(CANID::FromController, CANID::ToSlaveAll, CANID::Speed), speed);
		can_sender.send(CANID_generate(CANID::FromController, CANID::ToSlaveAll, CANID::Direction), direction);

		//DEBUG
		if(pc.readable()){
			pc.printf("mode:%2d  ", MRmode.get_now());
			pc.printf("sw_g:%d  ", sw_gerege.read());
			pc.printf("Rx:%3d  Ry:%3d  ", Rx, Ry);
			pc.printf("speed:%4.3f  dir:%1.3f  ", speed, direction);
			pc.printf("dist:%5.2f  ", walk_dist);
			pc.printf("\r\n");
		}
	}
}


void CANrcv(){
	if(can.read(rcvMsg)){
		unsigned int id = rcvMsg.id;
		if(!CANID_is_to(id, CANID::ToController))return;

		can_receiver.receive(id, rcvMsg.data);
		float can_data = can_receiver.get_data((CANID::DataType)(id&0x00f));

		if(CANID_is_type(id, CANID::MoveDistAvg)){
			pc.printf("rcvdist:%f  ", can_data);
			walk_dist += can_data;
			return;
		}

		if(CANID_is_type(id, CANID::AreaChange)){//エリアチェンジ要請
			MRmode.set((MRMode::Area)can_data);
			return;
		}
	}
}

void mode_command(){
	//課題突入
	if		(ps.BUTTON.BIT.MARU) 	MRmode.set_initial();
	else if (ps.BUTTON.BIT.BATU) 	MRmode.set(MRMode::SandDuneFront);
	else if (ps.BUTTON.BIT.SANKAKU) MRmode.set(MRMode::Tussock);
	else if (ps.BUTTON.BIT.SIKAKU)	MRmode.set(MRMode::Start2);

	int mrmode = MRmode.get_now();
	//段差越え:最初は前足だけ上げるモード。DOWN押すと後ろ足だけ上げるモード。次いでUP押すと初期モードに。
	if(mrmode==MRMode::SandDuneFront){
		if(ps.BUTTON.BIT.DOWN)MRmode.set(MRMode::SandDuneRear);
	}
	else if(mrmode==MRMode::SandDuneRear){
		if(ps.BUTTON.BIT.UP)MRmode.set_initial();
	}
	//登山中コマンド
	if(mrmode==MRMode::Start2){
		if(ps.BUTTON.BIT.UP)MRmode.set(MRMode::StartClimb1);
	}
	else if(mrmode==MRMode::StartClimb1){
		if(ps.BUTTON.BIT.DOWN)MRmode.set(MRMode::StartClimb2);
	}
	else if(mrmode==MRMode::StartClimb2){
		if(ps.BUTTON.BIT.LEFT)MRmode.set(MRMode::MountainArea);
	}

	//送信
	can_sender.send(CANID_generate(CANID::FromController, CANID::ToSlaveAll, CANID::Area), MRmode.get_now());

	//			   FR:leg_up&0x1			 RR:leg_up&0x2			   FL:leg_up&0x4			 RL:leg_up&0x8
	int leg_up = ((ps.BUTTON.BIT.R1)<<0) + ((ps.BUTTON.BIT.R2)<<1) + ((ps.BUTTON.BIT.L1)<<2) + ((ps.BUTTON.BIT.L2)<<3);
	can_sender.send(CANID_generate(CANID::FromController, CANID::ToSlaveAll, CANID::LegUp), leg_up);
	//	if(MRmode.get_now()==MRMode::SandDune && ps.BUTTON.BIT.SIKAKU)leg_up = 0xf;
}

void walk_command(float *speed, float *direction, float speed_max){
	float theta;
//	if((-ANALOG_MARGIN<Ry && Ry<ANALOG_MARGIN) && (-ANALOG_MARGIN<Rx && Rx<ANALOG_MARGIN)){
//		theta = 0;
//		Ry = 0;
//		Rx = 0;
//	}
//	else{
//		theta = atan2(Rx, Ry);
//	}
	stick_zero(&Rx, ANALOG_MARGIN);
	stick_zero(&Ry, ANALOG_MARGIN);
	theta = atan2(Rx, Ry);
	if(sqrt2(Rx, Ry)>ANALOG_MAX){
		Rx = ANALOG_MAX*sin(theta);
		Ry = ANALOG_MAX*cos(theta);
	}
	*speed = (Ry>=0? 1.0:-1.0)*(sqrt2(Rx, Ry)/(float)(ANALOG_MAX-ANALOG_MARGIN)) * speed_max;
	*direction = theta;

	switch(MRmode.get_now()){
	case MRMode::MountainArea:
		if(ps.BUTTON.BIT.UP)*speed = speed_max;
		break;
	}
}
