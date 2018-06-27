#include <pigpio.h>
#include <iostream>
#include <unistd.h>
#include <stdint.h>
#include "Sensor.h"
#include <stdio.h>


Sensor sensor;

using namespace std;

double Sdata[9];

FILE *fp;

void data();
int LoopNum=0;
int main()
{
	int s1,s2,m1,m2;
	gpioInitialise();
	sensor.getPi();
	sensor.hmcInit();
	sensor.mpuInit();
	sensor.GPSConfig();
	sensor.hmcSetConfigA(0x1c); //output rate and measurement configuration
	sensor.hmcSetConfigB(0x20);//Recommended Sensor Field Range 
	sensor.hmcSetMode(0x01);//0x01:single measurement mode(160Hz) 0x00:Continuous measurement mode
	sensor.mpuSetConfig();
	fp = fopen("/home/pi/sensorlog1.csv","w");
	gpioTime(0,&s1,&m1);
	fprintf(fp,"%d.%03d seconds\r\n",s1,m1/1000);
	while(1)
	{
		gpioSetTimerFunc(0,10,data);
	    if(LoopNum>=1500)
	    {
			break;
		}
	}
	while(1)
	{
		gpioSetTimerFunc(1,10,data2);
	}
	gpioTime(0,&s2,&m2);
	fprintf(fp,"%d.%03d seconds\r\n",s2,m2/1000);
	fclose(fp);
	sensor.pigpioStop();
}

void data()
{
	sensor.GetMotion9(Sdata);
	fprintf(fp,"%f;%f;%f;%f;%f;%f,%f,%f,%f\r\n",Sdata[0],Sdata[1],Sdata[2],Sdata[3],Sdata[4],Sdata[5],Sdata[6],Sdata[8],Sdata[7]);
	LoopNum++;
}

void data2()
{
	sensor.GetMotion9(Sdata);
	fprintf(fp,"%f;%f;%f;%f;%f;%f,%f,%f,%f\r\n",Sdata[0],Sdata[1],Sdata[2],Sdata[3],Sdata[4],Sdata[5],Sdata[6],Sdata[8],Sdata[7]);
}

//How To Use--------------------------------------------------------------------------------
//I2C connect check: $ sudo i2cdetect -y 1
//kill deamon $ sudo killall pigpiod
//compile:$ g++ -Wall -pthread -o main Sensor.cpp  main.cpp -lpigpio -lpigpiod_if2 -lrt
//exe active: $ sudo ./main
