/*
 * PhotoelectricSensor.h
 *
 *  Created on: 2019/01/15
 *      Author: mutsuro
 */

#ifndef PHOTOELECTRICSENSOR_H_
#define PHOTOELECTRICSENSOR_H_

#include "mbed.h"

/*
 * 光電センサ:E18-D80NK
 *
 * range by screw
 * cw:large
 * ccw:small
 *
 * max:工房の机の高さくらい
 * min:2~3cm
 */

class PhotoelectricSensor{
public:
	PhotoelectricSensor(PinName input, bool use_timer);
	void sensing();
	int read();
	bool is_rising();
	float get_ontime();
	unsigned int get_counter();
private:
	DigitalIn *Input;
	Timer *tm_kouden;
	bool use_timer;
	short now;
	short prev;
	unsigned int counter;
};


#endif /* PHOTOELECTRICSENSOR_H_ */
