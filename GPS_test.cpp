#include <pigpio.h>
#include <iostream>
#include <unistd.h>
#include <stdint.h>
#include "Sensor.h"
#include <stdio.h>

Sensor sensor;

int main()
{
	float GPSData[2];
	gpioInitialise();
	sensor.GPSGetData(GPSData);
   
}
/* how to compile

g++ -Wall -pthread -o GPS Sensor.cpp GPS_test.cpp -lm -lgps -lpigpio -lrt

*/