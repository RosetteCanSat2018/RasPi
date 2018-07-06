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
	sensor.GPSread();
	sensor.GPSGetLine(GPSData);
   
}
