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
	sensor.GPSConfig();
    while(1) 
    {
		sensor.GPSGetData(GPSData);
        printf("I'm at %f, %f\n", GPSData[0],GPSData[1] );

    }
}
