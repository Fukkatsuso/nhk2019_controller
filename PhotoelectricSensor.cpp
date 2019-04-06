/*
 * PhotoelectricSensor.cpp
 *
 *  Created on: 2019/01/15
 *      Author: mutsuro
 */

#include "PhotoelectricSensor.h"


PhotoelectricSensor::PhotoelectricSensor(PinName input, bool use_timer)
{
	Input = new DigitalIn(input);
	Input->mode(PullUp);
	now = prev = 0;
	this->use_timer = use_timer;
	if(use_timer)tm_kouden = new Timer;
	else counter = 0;
}


void PhotoelectricSensor::sensing()
{
	prev = now;
	now = 1-(Input->read());
	if(use_timer){
		if(is_rising()){
			tm_kouden->reset();
			tm_kouden->start();
		}
		else if(now==0){
			tm_kouden->stop();
			tm_kouden->reset();
		}
	}
	else{
		if(now)counter++;
		else if(counter > 0)counter--;
	}
}

/*
 * 検知:1
 * 非検知:0
 */
int PhotoelectricSensor::read()
{
	return now;
}

bool PhotoelectricSensor::is_rising()
{
	return (prev==0 && now==1);
}

float PhotoelectricSensor::get_ontime()
{
	if(use_timer)tm_kouden->read();
	else return 0;
}

unsigned int PhotoelectricSensor::get_counter()
{
	if(!use_timer)return counter;
	else return 0;
}
