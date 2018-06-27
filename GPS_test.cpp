#include <pigpio.h>
#include <iostream>
#include <unistd.h>
#include <stdint.h>
#include "Sensor.h"
#include <stdio.h>

Sensor sensor;
int i=0;
int main()
{
	float GPSData[2];
	sensor.GPSConfig();
    while(1) 
    {
		sensor.GPSGetData(GPSData);
        i++;
        if(i<1000)
        {
			break;
		}

    }
}
