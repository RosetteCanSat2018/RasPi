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

#define MPU6050_ADDRS                    0x68
#define MPU6050_RA_ACCEL_XOUT_H         0x3B
#define HMC5883L_ADDRS                  0x1E
//#define HMC5883L_i2c                  0x3D
#define CONFIG_A_REG                    0x00
#define CONFIG_B_REG                    0x01
#define MODE_REG                        0x02


void Sensor::getPi()
{
	gpioInitialise();
	MPU6050_i2c = i2cOpen(1,MPU6050_ADDRS,0);
	HMC5883L_i2c = i2cOpen(1 ,HMC5883L_ADDRS,0);
}
//Initialize---------------------------------
//MPU6050
void Sensor::mpuInit()
{
	i2cWriteWordData(MPU6050_i2c,0x6B,0x80);
	i2cWriteWordData(MPU6050_i2c,0x6B,0x00);
	i2cWriteWordData(MPU6050_i2c,0x6A,0x07);
	i2cWriteWordData(MPU6050_i2c,0x6A,0x00);
}
//HMC5883L
void Sensor::hmcInit()
{
	i2cWriteWordData(HMC5883L_i2c,0x00,0x70); //8-Average,15Hzdefault,normalmesurement
	i2cWriteWordData(HMC5883L_i2c,0x01,0xA0); // Gain=5
	i2cWriteWordData(HMC5883L_i2c,0x02,0x00); // Continuous-measurement mode
}
//Config--------------------------------------
//MPU6050
void Sensor::mpuSetConfig()
{
	i2cWriteWordData(MPU6050_i2c,0x1A,0x00); //CONFIG
	i2cWriteWordData(MPU6050_i2c,0x1B,0x18); //+-2000/s
	i2cWriteWordData(MPU6050_i2c,0x1C,0x18); //+-16g
	//i2cWriteWordData(MPU6050_i2c,0x24,0x0D);
}
//2000:0x18 1000:0x10 500:0x08 250:0x00
//+-16g:0x18 +-8g:0x10 +-4g:0x08 +-2g:0x00
//HMC5883L
void Sensor::hmcSetConfigA(char config)
	{
		char cmd[2];
		cmd[0] = CONFIG_A_REG;
		cmd[1] = config;
		i2cWriteWordData(HMC5883L_i2c,cmd[0],cmd[1]);
	}
	void Sensor::hmcSetConfigB(char config)
	{
		char cmd[2];
		cmd[0] = CONFIG_B_REG;
		cmd[1] = config;
		i2cWriteWordData(HMC5883L_i2c,cmd[0],cmd[1]);
	}
	void Sensor::hmcSetMode(char config)
	{
		char cmd[2];
		cmd[0] = MODE_REG;
		cmd[1] = config;
		i2cWriteWordData(HMC5883L_i2c,cmd[0],cmd[1]);
	}

//getData---------------------------------------
//MPU6050
void Sensor::mpuGetMotion6(double ACGY[6])
{
	char data[12];
	int16_t output[6];
	data[0]= i2cReadWordData(MPU6050_i2c,0x3B);
	data[1]= i2cReadWordData(MPU6050_i2c,0x3C);
	data[2]= i2cReadWordData(MPU6050_i2c,0x3D);
	data[3]= i2cReadWordData(MPU6050_i2c,0x3E);
	data[4]= i2cReadWordData(MPU6050_i2c,0x3F);
	data[5]= i2cReadWordData(MPU6050_i2c,0x40);
	data[6]= i2cReadWordData(MPU6050_i2c,0x43);
	data[7]= i2cReadWordData(MPU6050_i2c,0x44);
	data[8]= i2cReadWordData(MPU6050_i2c,0x45);
	data[9]= i2cReadWordData(MPU6050_i2c,0x46);
	data[10]= i2cReadWordData(MPU6050_i2c,0x47);
	data[11]= i2cReadWordData(MPU6050_i2c,0x48);
	//cout<<data[0]<<endl;
	for(int i =0 ;i < 6;i++)
	{
		output[i] = int16_t((unsigned char)data[i*2]<<8 | (unsigned char) data[i*2+1]);
	}
	
	ACGY[0] = -1*(double)output[0]/2048.0*9.81;
	ACGY[1] = -1*(double)output[1]/2048.0*9.81;
	ACGY[2] = -1*(double)output[2]/2048.0*9.81; //+-16g:2048.0 +-8g:4096.0 +-4g:8192.0 +-2g:16384.0 
	ACGY[3] = (double)output[3]/939.7;
	ACGY[4] = (double)output[4]/939.7;
	ACGY[5] = (double)output[5]/939.7; //2000/s:939.7 1000/s:1879.3 500/s:3752.9 250/s:7505.7
 	
}
//HMC5883L
void  Sensor::hmcGetXYZ(double Mg[3])
{
	char data[6];
	int16_t output[3];
	
	//i2cWriteWordData(HMC5883L_i2c,0x3c,0x01); 
	data[0]= i2cReadWordData(HMC5883L_i2c,0x03);
	data[1]= i2cReadWordData(HMC5883L_i2c,0x04);
	data[2]= i2cReadWordData(HMC5883L_i2c,0x05);
	data[3]= i2cReadWordData(HMC5883L_i2c,0x06);
	data[4]= i2cReadWordData(HMC5883L_i2c,0x07);
	data[5]= i2cReadWordData(HMC5883L_i2c,0x08);
	for(int i =0 ;i < 3;i++)
	{
		output[i] = int16_t((unsigned char)data[i*2]<<8 | (unsigned char) data[i*2+1]);
	}
	Mg[0] = (double)output[0]/1090.0;
	Mg[1] = (double)output[1]/1090.0;
	Mg[2] = (double)output[2]/1090.0;
	
}
//GPS
void Sensor::GPSGetData(float data[2])
{
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
					printf("latitude: %f, longitude: %f", gps_data.fix.latitude, gps_data.fix.longitude); //EDIT: Replaced tv.tv_sec with gps_data.fix.time
					data[0] = gps_data.fix.latitude;
					data[1] = gps_data.fix.longitude;
				}
				else
				{
					printf("no GPS data available\n");
				}
			}
		}
		cout << "out1" << endl;
		sleep(3);
	}
	cout << "out2" << endl;
	//return 0;
}
//Finish---------------------------------------------
void Sensor::pigpioStop()
{
	i2cClose(MPU6050_i2c);
	i2cClose(HMC5883L_i2c);
	serClose(ser);
}

#endif