#include <pigpio.h>
#include <iostream>
#include <unistd.h>
#include <stdint.h>
#include "Sensor.h"
#include <stdio.h>
#include <math.h>

using namespace std;

Sensor sensor;

#define TIMER 3000 //[msec]
#define COUNTER 10

float GPS[2], GPS_prev[2];
int GPS_0_counter, GPS_1_counter;

void func();

int main()
{
	gpioInitialise();
	GPS_prev[0] = 0;
	GPS_prev[1] = 0;

	while (1)
	{
		gpioSetTimerFunc(0, TIMER, func);
		//if (GPS_0_counter > COUNTER && GPS_1_counter > COUNTER && alt_counter > COUNTER)
		if (GPS_0_counter > COUNTER && GPS_1_counter > COUNTER)
		{
			cout << "landing" << endl;
			gpioSetTimerFunc(0, TIMER, NULL);
			break;
		}
	}
}

void func()
{
	sensor.GPSGetData_f(GPS);

	if (fabsf(GPS[0] - GPS_prev[0]) <= 0.0002)
	{
		GPS_0_counter++;
	}
	else if (fabsf(GPS[0] - GPS_prev[0]) > 0.0002)
	{
		GPS_0_counter = 0;
	}
	if (fabsf(GPS[1] - GPS_prev[1]) <= 0.0002)
	{
		GPS_1_counter++;
	}
	else if (fabsf(GPS[1] - GPS_prev[1]) > 0.0002)
	{
		GPS_1_counter = 0;
	}/*
	 if (fabsf(alt_prev - alt) <= 5.0)
	 {
	 alt_counter++;
	 }
	 else if (fabsf(alt_prev - alt) > 5.0)
	 {
	 alt_counter = 0;
	 }
	 */
	GPS_prev[0] = GPS[0];
	GPS_prev[1] = GPS[1];
	//alt_prev = alt;
	//cout << GPS_0_counter << ";" << GPS_1_counter << ";" << alt_counter << endl;
	cout << GPS[0] << ";" << GPS[1] << ";" << GPS_0_counter << ";" << GPS_1_counter << endl;
	//cout << alt << ";"<< alt_counter << endl;
}
