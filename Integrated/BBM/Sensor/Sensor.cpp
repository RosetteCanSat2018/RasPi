#ifndef _Sensor_H_
#define _Sensor_H_

#include <pigpio.h>
#include "Sensor.h"
#include <unistd.h>
#include <stdint.h>
#include <math.h>
#include <string.h>
#include <iostream>
#include <gps.h>
#include <string>
#include <algorithm>

#define MPL3115A2_ADDRS                 0x60

//Config------------------------------------------------------------------------------------------------------------------
//MPL3115A2
	void Sensor::mplSetConfig()
	{
		MPL3115A2_i2c = i2cOpen(1, MPL3115A2_ADDRS, 0);
		i2cWriteWordData(MPL3115A2_i2c, 0x26, 0xB8); //Active mode, OSR = 10, Altimeter mode
		i2cWriteWordData(MPL3115A2_i2c, 0x13, 0x07); //Data ready event enabled for altitude, pressure, temperature
		i2cWriteWordData(MPL3115A2_i2c, 0x26, 0x01); //Data ready event enabled for altitude, pressure, temperature
	}
//getData------------------------------------------------------------------------------------------------------------------
//MPL3115A2
double Sensor::mplGetALT(double sea_pressure)
{
	char data[6];
	i2cReadI2CBlockData(MPL3115A2_i2c, 0x00, data, 6);
	int pres = ((data[1] * 65536) + (data[2] * 256 + (data[3] & 0xF0))) / 16;
	double pressure = (pres / 4.0) / 100.0;
	int temp = ((data[4] * 256) + (data[5] & 0xF0)) / 16;
	float cTemp = (temp / 16.0);
	double alt = ((pow((sea_pressure / pressure), (1 / 5.257)) - 1)*(cTemp + 273.15)) / 0.0065;
	return alt;
}
//GPS
void Sensor::GPSGetData(char c_data[2])
{
	GPS_handle = serOpen("/dev/ttyS0", 9600, 0);
	float data[2];
	string s_data;
	//struct timeval tv;
	struct gps_data_t gps_data;
	if ((rc = gps_open("localhost", "2947", &gps_data)) == -1)
	{
		printf("code: %d, reason: %s\n", rc, gps_errstr(rc));
		//return EXIT_FAILURE;
	}
	gps_stream(&gps_data, WATCH_ENABLE | WATCH_JSON, NULL);

	while (1)
	{
		/* wait for 2 seconds to receive data */
		if (gps_waiting(&gps_data, 2000000))
		{
			/* read data */
			if ((rc = gps_read(&gps_data)) == -1)
			{
				printf("error occured reading gps data. code: %d, reason: %s\n", rc, gps_errstr(rc));
			}
			else
			{
				/* Display data from the GPS receiver. */
				if ((gps_data.status == STATUS_FIX) &&
					(gps_data.fix.mode == MODE_2D || gps_data.fix.mode == MODE_3D) &&
					!isnan(gps_data.fix.latitude) &&
					!isnan(gps_data.fix.longitude))
				{
					//gettimeofday(&tv, NULL); EDIT: tv.tv_sec isn't actually the timestamp!
					//printf("latitude: %f, longitude: %f", gps_data.fix.latitude, gps_data.fix.longitude); //EDIT: Replaced tv.tv_sec with gps_data.fix.time
					data[0] = gps_data.fix.latitude;
					data[1] = gps_data.fix.longitude;
					for(int i=0; i<2; i++){
					s_data = to_string(data[i]);
					s_data.copy(c_data,2);
				}
				}
				else
				{
					printf("no GPS data available\n");
				}
			}
		}
	}
	//return 0;
	serClose(GPS_handle);
}

void Sensor::GPSGetData_f(float data[2])
{
	GPS_handle = serOpen("/dev/ttyS0", 9600, 0);
	//struct timeval tv;
	struct gps_data_t gps_data;
	if ((rc = gps_open("localhost", "2947", &gps_data)) == -1)
	{
		printf("code: %d, reason: %s\n", rc, gps_errstr(rc));
		//return EXIT_FAILURE;
	}
	gps_stream(&gps_data, WATCH_ENABLE | WATCH_JSON, NULL);

	int count = 0;

	while (1)
	{
		/* wait for 2 seconds to receive data */
		if (gps_waiting(&gps_data, 2000000))
		{
			/* read data */
			if ((rc = gps_read(&gps_data)) == -1)
			{
				printf("error occured reading gps data. code: %d, reason: %s\n", rc, gps_errstr(rc));

			}
			else
			{
				/* Display data from the GPS receiver. */
				if ((gps_data.status == STATUS_FIX) &&
					(gps_data.fix.mode == MODE_2D || gps_data.fix.mode == MODE_3D) &&
					!isnan(gps_data.fix.latitude) &&
					!isnan(gps_data.fix.longitude))
				{
					//gettimeofday(&tv, NULL); EDIT: tv.tv_sec isn't actually the timestamp!
					//printf("latitude: %f, longitude: %f", gps_data.fix.latitude, gps_data.fix.longitude); //EDIT: Replaced tv.tv_sec with gps_data.fix.time
					data[0] = gps_data.fix.latitude;
					data[1] = gps_data.fix.longitude;
					break;
				}
				else
				{
					
					//printf("no GPS data available\n");

					//if(count == 5){break;}

					//count++;
				}
			}
		}
	}
	serClose(GPS_handle);
}
//Finish----------------------------------------------------------------------------------------------------------------------------------------------------------
void Sensor::pigpioStop()
{
	i2cClose(MPL3115A2_i2c);
	gpioTerminate();
}

#endif
