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
#include "Walk/CANs/CANNavigation.h"


CANMessage rcvMsg;
CANSender can_sender(&can);
CANReceiver can_receiver(&can);
CANNavigation can_navi(&can);
MRMode MRmode(
		MRMode::WaitGobiUrtuu,
//		MRMode::WaitMountainUrtuu,

		&can_sender);


void CANrcv();
void mode_operate_command();
void walk_operate_command(float *speed, float *direction, float speed_max);


int main(){
	short walk_dist_front = 0;
	short walk_dist_rear = 0;
	unsigned char kouden_sanddune_front = 0;
	unsigned char kouden_sanddune_rear = 0;
	short walk_dist = 0;//機体重心の総移動距離

	float speed = 0;
	float direction = 0;

	unsigned int counter_sanddune_front = 0;
	unsigned int counter_sanddune_rear = 0;

	can.frequency(1000000);
	can.attach(&CANrcv, CAN::RxIrq);
	wait_ms(300); //全ての基板の電源が入るまで待つ
	pc.baud(921600);

	MRmode.set_sensors(&sw_gerege, &kouden_urtuu2);

	while(1){
		AdjustCycle(5000);

		//PSコン操作時のみ
//		psCommand();
//		if(ps.ANALOG_MODE){
//			walk_operate_command(&speed, &direction, speed_max);
//			mode_operate_command();
//		}

		//自動ver.
		//Area, speed, direction計算
		//LegUp計算
		MRmode.plan(
				//どれか1つだけコメントアウト
				can_navi.get_status(),
//				CANNavigation::Walk,
//				CANNavigation::MountainArea,

				-((float)can_navi.get_angle())*M_PI/180.0,
				walk_dist,
				counter_sanddune_front, //kouden_sanddune_front,
				counter_sanddune_rear //kouden_sanddune_rear
				);

		//Gerege掲げる
		if(MRmode.get_now() >= MRMode::Uukhai) cyl_gerege.write(1);
		else cyl_gerege.write(0);

		//Area送信
		MRmode.send();

		//速度ベクトル送信
		direction = MRmode.get_direction();
		speed = MRmode.get_speed();
		can_sender.send_velocity_vector(
				CANID::generate(CANID::FromController, CANID::ToSlaveAll, CANID::VelocityVector), direction, speed);

		//歩行量計算+送信
		//前脚からの情報
		walk_dist_front = can_receiver.get_move_position_front_dist();
		kouden_sanddune_front = can_receiver.get_move_position_front_kouden_sanddune();
		//後脚からの情報
		walk_dist_rear = can_receiver.get_move_position_rear_dist();
		kouden_sanddune_rear = can_receiver.get_move_position_rear_kouden_sanddune();
		//Masterへ送信
		walk_dist = walk_dist_front + walk_dist_rear;
		can_navi.send(walk_dist, kouden_sanddune_front, kouden_sanddune_rear);

		//光電センサの値更新
		counter_sanddune_front = counter_update(counter_sanddune_front, kouden_sanddune_front);
		counter_sanddune_rear = counter_update(counter_sanddune_rear, kouden_sanddune_rear);
		if(MRmode.get_now() < MRMode::SandDuneFront)counter_sanddune_front = 0;
		if(MRmode.get_now() < MRMode::SandDuneRear)counter_sanddune_rear = 0;

		//DEBUG
		if(pc.readable()){
				//シリンダ操作
//				int ch = pc.getc();
//				if(ch=='c')cyl_gerege.write(1);
//				else if(ch=='d')cyl_gerege.write(0);
			pc.printf("mode:%2d  ", MRmode.get_now());
			pc.printf("sw_g:%d  ", sw_gerege.read());
			pc.printf("cyl:%1d  ", cyl_gerege.read());
//			pc.printf("Rx:%3d  Ry:%3d  ", Rx, Ry);
			pc.printf("speed:%4.3f  dir:%1.3f  ", speed, direction);
			pc.printf("dist:[%5d][%5d]  ", walk_dist_front, walk_dist_rear);
			pc.printf("kouden[%1d][%1d][%1d]  ", kouden_sanddune_front, kouden_sanddune_rear, kouden_urtuu2.read());
			pc.printf("navi[%d][%d]  ", can_navi.get_angle(), can_navi.get_status());
//			pc.printf("cnt_sdr:%d  ", counter_sanddune_rear);
			pc.printf("\r\n");
		}
	}
}


void CANrcv(){
	if(can.read(rcvMsg)){
		unsigned int id = rcvMsg.id;
		if(!CANID::is_to(id, CANID::ToController))return;

		if(CANID::is_from(id, CANID::FromMaster)){
			can_navi.receive(rcvMsg);
			return;
		}
		//Slaveとの通信
		can_receiver.receive(rcvMsg);

		if(CANID::is_type(id, CANID::AreaChange)){//エリアチェンジ要請
			MRmode.set((MRMode::Area)can_receiver.get_area_change());
		}
	}
}

void mode_operate_command(){
	//課題突入
	if		(ps.BUTTON.BIT.MARU) 	MRmode.set(MRMode::GobiArea);
	else if (ps.BUTTON.BIT.BATU) 	MRmode.set(MRMode::SandDuneFront);
	else if (ps.BUTTON.BIT.SANKAKU) MRmode.set(MRMode::Tussock);
	else if (ps.BUTTON.BIT.SIKAKU)	MRmode.set(MRMode::Start2);

	int mrmode = MRmode.get_now();
	//段差越え:最初は前足だけ上げるモード。DOWN押すと後ろ足だけ上げるモード。次いでUP押すと初期モードに。
	if(mrmode==MRMode::SandDuneFront){
		if(ps.BUTTON.BIT.DOWN)MRmode.set(MRMode::SandDuneRear);
	}
	else if(mrmode==MRMode::SandDuneRear){
		if(ps.BUTTON.BIT.UP)MRmode.set(MRMode::GobiArea);
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
	can_sender.send_area(CANID::generate(CANID::FromController, CANID::ToSlaveAll, CANID::Area), MRmode.get_now());

	//					   FR:leg_up&0x1			 FL:leg_up&0x2				RR:leg_up&0x4			 RL:leg_up&0x8
	unsigned char leg_up = ((ps.BUTTON.BIT.R1)<<0) + ((ps.BUTTON.BIT.L1)<<1) + ((ps.BUTTON.BIT.R2)<<2) + ((ps.BUTTON.BIT.L2)<<3);
	can_sender.send_leg_up(CANID::generate(CANID::FromController, CANID::ToSlaveAll, CANID::LegUp), leg_up);
	//	if(MRmode.get_now()==MRMode::SandDune && ps.BUTTON.BIT.SIKAKU)leg_up = 0xf;
}

void walk_operate_command(float *speed, float *direction, float speed_max){
	float theta;
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
