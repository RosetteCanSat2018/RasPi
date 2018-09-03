#ifndef _Servo_H_
#define _Servo_H_

#include <pigpio.h>
#include "Servo.h"
#include "stdio.h"

void Servo::SetServo(unsigned int _servo1_gpio, unsigned int _servo2_gpio)
{
	servo1_gpio = _servo1_gpio;
	servo2_gpio = _servo2_gpio;

	gpioSetMode(servo1_gpio, 1);
	gpioSetMode(servo2_gpio, 1);
}

void Servo::MoveServo(int puddledegree)
{
	double pulsewidth = 1500 + puddledegree * 500 / 60; // +60deg:2ms, -60deg:1ms, +90deg:2.3ms, -90deg:0.7ms
	gpioServo(servo1_gpio, pulsewidth);//frequency:50Hz
	//printf("pulsewidth = %f\r\n",pulsewidth);
	gpioServo(servo2_gpio, pulsewidth);
}

#endif
