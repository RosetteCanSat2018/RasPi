#include <pigpio.h>
#include <iostream>
#include <unistd.h>
#include <stdint.h>
#include "Sensor.h"
#include <stdio.h>
#include <math.h>


#define TIMER 3000 //[msec]
#define COUNTER 10
#define SEA_LEVEL_PRESSURE 1007 //[hPa]

using namespace std;

Sensor sensor;

float GPS[2], GPS_prev[2];
double alt, alt_prev, alt_dec;
int GPS_0_counter, GPS_1_counter, alt_counter;

void func();

int main()
{
	gpioInitialise();
	sensor.mplSetConfig();
	GPS_prev[0] = 0;
	GPS_prev[1] = 0;
	alt_prev = 0;

	while (1)
	{
		alt_dec = sensor.mplGetALT(SEA_LEVEL_PRESSURE);
		cout << alt_dec << endl;
		if (alt_dec < 30)
		{
			gpioSetTimerFunc(0, TIMER, func);
			if (GPS_0_counter > COUNTER && GPS_1_counter > COUNTER && alt_counter > COUNTER)
			{
				cout << "landing" << endl;
				gpioSetTimerFunc(0, TIMER, NULL);
				break;
			}
		}
		else if (alt_dec>30)
		{
			cout << "not landing mode" << endl;
			GPS_0_counter = 0;
			GPS_1_counter = 0;
			alt_counter = 0;
		}
		sleep(3);
	}
}
void func()
{
	alt = sensor.mplGetALT(SEA_LEVEL_PRESSURE);
	sensor.GPSGetData_f(GPS);

	if (fabsf(GPS[0] - GPS_prev[0]) = 0)
	{
		GPS_0_counter++;
	}
	else if (fabsf(GPS[0] - GPS_prev[0]) > 0)
	{
		GPS_0_counter = 0;
		GPS_1_counter = 0;
		alt_counter = 0;
	}
	if (fabsf(GPS[1] - GPS_prev[1]) = 0)
	{
		GPS_1_counter++;
	}
	else if (fabsf(GPS[1] - GPS_prev[1]) > 0)
	{
		GPS_0_counter = 0;
		GPS_1_counter = 0;
		alt_counter = 0;
	}
	if (fabsf(alt_prev - alt) <= 5.0)
	{
		alt_counter++;
	}
	else if (fabsf(alt_prev - alt) > 5.0)
	{
		GPS_0_counter = 0;
		GPS_1_counter = 0;
		alt_counter = 0;
	}
