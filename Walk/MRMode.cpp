/*
 * MRMode.cpp
 *
 *  Created on: 2019/03/03
 *      Author: mutsuro
 */


#include "MRMode.h"

MRMode::MRMode(enum Area area_initial, CANSender *can_sender)
{
	this->can_sender = can_sender;
	now = area_initial;
}

void MRMode::send()
{
	can_sender->send(CANID_generate(CANID::FromMaster, CANID::ToSlaveAll, CANID::Area), now);
}

int MRMode::get_now()
{
	return now;
}
