#include <pigpio.h>
#include <iostream>
#include <unistd.h>
#include <stdint.h>
#include "Sensor.h"
#include <stdio.h>

Sensor sensor;

int main()
{
	char GPSData[2]
	sensor.GPSConfig();
    while(1) 
    {
        if(sensor.GPSGetData(GPSdata)) 
			{
				printf("I'm at %f, %f\n", GPSdata[0],GPSdata[1] );
			} 
		else 
			{
				printf("Oh Dear! No lock :(\n");
			}
    }
}
