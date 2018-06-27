#ifndef _Sensor_H_
#define _Sensor_H_

#include <pigpio.h>
#include "Sensor.h"
#include <pigpiod_if2.h>
#include <unistd.h>
#include <stdint.h>
#include <iostream>
#include <string.h>
#include <math.h>


#define MPU6050_ADDRS                   0x68
#define MPU6050_RA_ACCEL_XOUT_H         0x3B
#define HMC5883L_ADDRS                  0x1E
//#define HMC5883L_i2c                  0x3D
#define CONFIG_A_REG                    0x00
#define CONFIG_B_REG                    0x01
#define MODE_REG                        0x02
#define GPSgpio                         15
#define GPSbaud                         9600

void Sensor::getPi()
{
	
	pi = pigpio_start(0,0);
	MPU6050_i2c = i2c_open(pi,1,MPU6050_ADDRS,0);
	HMC5883L_i2c = i2c_open(pi, 1 ,HMC5883L_ADDRS,0);
}
//Initialize---------------------------------
//MPU6050
void Sensor::mpuInit()
{
	i2c_write_word_data(pi, MPU6050_i2c,0x6B,0x80);
	i2c_write_word_data(pi, MPU6050_i2c,0x6B,0x00);
	i2c_write_word_data(pi, MPU6050_i2c,0x6A,0x07);
	i2c_write_word_data(pi, MPU6050_i2c,0x6A,0x00);
}
//HMC5883L
void Sensor::hmcInit()
{
	i2c_write_word_data(pi,HMC5883L_i2c,0x3c,0x70); //8-Average,15Hzdefault,normalmesurement
	i2c_write_word_data(pi,HMC5883L_i2c,0x3c,0xA0); // Gain=5
	i2c_write_word_data(pi,HMC5883L_i2c,0x3c,0x00); // Continuous-measurement mode
}
//Config--------------------------------------
//MPU6050
void Sensor::mpuSetConfig()
{
	i2c_write_word_data(pi, MPU6050_i2c,0x1A,0x07); //CONFIG
	i2c_write_word_data(pi, MPU6050_i2c,0x1B,0x18); //+-2000/s
	i2c_write_word_data(pi, MPU6050_i2c,0x1C,0x18); //+-16g
}
//2000:0x18 1000:0x10 500:0x08 250:0x00
//+-16g:0x18 +-8g:0x10 +-4g:0x08 +-2g:0x00
//HMC5883L
void Sensor::hmcSetConfigA(char config)
{
	char cmd[2];
	cmd[0] = CONFIG_A_REG;
	cmd[1] = config;
	i2c_write_word_data(pi,HMC5883L_i2c,cmd[0],cmd[1]);
}
void Sensor::hmcSetConfigB(char config)
{
	char cmd[2];
	cmd[0] = CONFIG_B_REG;
	cmd[1] = config;
	i2c_write_word_data(pi,HMC5883L_i2c,cmd[0],cmd[1]);
}
void Sensor::hmcSetMode(char config)
{
	char cmd[2];
	cmd[0] = MODE_REG;
	cmd[1] = config;
	i2c_write_word_data(pi,HMC5883L_i2c,cmd[0],cmd[1]);
}
void Sensor::GPSConfig()
{
	set_mode(pi,GPSgpio,PI_INPUT);
	bb_serial_read_open(pi,GPSgpio,GPSbaud,8);
}
//getData---------------------------------------
void Sensor::GetMotion9(double Sdata[9])
{
//MPU6050---------------------------------------
	char data[18];
	int16_t output[9];
	data[0]= i2c_read_word_data(pi,MPU6050_i2c,0x3B);
	data[1]= i2c_read_word_data(pi,MPU6050_i2c,0x3C);
	data[2]= i2c_read_word_data(pi,MPU6050_i2c,0x3D);
	data[3]= i2c_read_word_data(pi,MPU6050_i2c,0x3E);
	data[4]= i2c_read_word_data(pi,MPU6050_i2c,0x3F);
	data[5]= i2c_read_word_data(pi,MPU6050_i2c,0x40);
	data[6]= i2c_read_word_data(pi,MPU6050_i2c,0x43);
	data[7]= i2c_read_word_data(pi,MPU6050_i2c,0x44);
	data[8]= i2c_read_word_data(pi,MPU6050_i2c,0x45);
	data[9]= i2c_read_word_data(pi,MPU6050_i2c,0x46);
	data[10]= i2c_read_word_data(pi,MPU6050_i2c,0x47);
	data[11]= i2c_read_word_data(pi,MPU6050_i2c,0x48);
	for(int i =0 ;i < 6;i++)
	{
		output[i] = int16_t((unsigned char)data[i*2]<<8 | (unsigned char) data[i*2+1]);
	}
	
	Sdata[0] = -1*(float)output[0]/2048.0*9.81;
	Sdata[1] = -1*(float)output[1]/2048.0*9.81;
	Sdata[2] = -1*(float)output[2]/2048.0*9.81; //+-16g:2048.0 +-8g:4096.0 +-4g:8192.0 +-2g:16384.0 
	Sdata[3] = (float)output[3]/939.7;
	Sdata[4] = (float)output[4]/939.7;
    Sdata[5] = (float)output[5]/939.7; //2000/s:939.7 1000/s:1879.3 500/s:3752.9 250/s:7505.7
//HMC5883L-------------------------------------------

	data[12]= i2c_read_word_data(pi,HMC5883L_i2c,0x03);
	data[13]= i2c_read_word_data(pi,HMC5883L_i2c,0x04);
	data[14]= i2c_read_word_data(pi,HMC5883L_i2c,0x05);
	data[15]= i2c_read_word_data(pi,HMC5883L_i2c,0x06);
	data[16]= i2c_read_word_data(pi,HMC5883L_i2c,0x07);
	data[17]= i2c_read_word_data(pi,HMC5883L_i2c,0x08);
	for(int i =6 ;i < 9;i++)
	{
		output[i] = int16_t((unsigned char)data[i*2]<<8 | (unsigned char) data[i*2+1]);
	}
	Sdata[6] = (double)output[6]/1090.0;
	Sdata[7] = (double)output[7]/1090.0;
	Sdata[8] = (double)output[8]/1090.0;
}

void Sensor::GPSGetData(float data[2])
{
	float time;
    char ns, ew;
    int lock;
	
    while(1)
    {
		bb_serial_read(pi,GPSgpio,msg,256);
		if(sscanf(msg, "GPGGA,%f,%f,%c,%f,%c,%d", &time, &latitude, &ns, &longitude, &ew, &lock) >= 1) 
			{ 
				if(!lock)
					{
						longitude = 0.0;
						latitude = 0.0;
						return 0;        
					} 
				else 
					{
						if(ns == 'S') {    latitude  *= -1.0; }
						if(ew == 'W') {    longitude *= -1.0; }
						float degrees = GPStrunc(latitude / 100.0f);
						float minutes = latitude - (degrees * 100.0f);
						latitude = degrees + minutes / 60.0f;    
						degrees = GPStrunc(longitude / 100.0f * 0.01f);
						minutes = longitude - (degrees * 100.0f);
						longitude = degrees + minutes / 60.0f;
						data[1] = latitude;
						data[0] = longitude;
						printf("%f,%f"data[0],data[1]);
					}
			}
}


float Sensor::GPStrunc(float v) 
{
    if(v < 0.0) 
		{
			v*= -1.0;
			v = floor(v);
			v*=-1.0;
		} 
	else 
		{
			v = floor(v);
		}
    return v;
}

//Finish---------------------------------------------
void Sensor::pigpioStop()
{
	i2c_close(pi,MPU6050_i2c);
	i2c_close(pi,HMC5883L_i2c);
}



#endif
