#include <pigpio.h>
#include <iostream>
#include <unistd.h>
#include <stdint.h>
#include "Sensor.h"
#include <stdio.h>

using namespace std;

Sensor sensor;

double Mg[3],ACGY[6];
int s1,s2,m1,m2,i;
float GPSdata[2];

FILE *fp;

void data();

int main()
{
	i=0;
	gpioInitialise();
	sensor.getPi();
	sensor.hmcInit();
	sensor.mpuInit();
	sensor.hmcSetConfigA(0x18); //output rate and measurement configuration
	sensor.hmcSetConfigB(0x20);//Recommended Sensor Field Range 
	sensor.hmcSetMode(0x00);//0x01:single measurement mode(160Hz) 0x00:Continuous measurement mode
	sensor.mpuSetConfig();
	fp = fopen("/home/pi/Sensor/sensorlog.csv","w");
	gpioTime(0,&s1,&m1);
	fprintf(fp,"%d.%03d seconds\r\n",s1,m1/1000);
	while(1)
	{
		gpioSetTimerFunc(0,10,data);
	    if(i>=1000)
	    {
			break;
		}
	}
	//sensor.GPSGetData(GPSdata);
	gpioTime(0,&s2,&m2);
	fprintf(fp,"%d.%03d seconds\r\n",s2,m2/1000);
	fclose(fp);
	sensor.pigpioStop();
}

void data()
{
	sensor.hmcGetXYZ(Mg);
	sensor.mpuGetMotion6(ACGY);
	fprintf(fp,"%lf;%lf;%lf;%lf;%lf;%lf,%lf,%lf,%lf\r\n",Mg[2],-1*Mg[0],Mg[1],ACGY[0],ACGY[1],ACGY[2],ACGY[3],ACGY[4],ACGY[5]);
	i++;
}



//How To Use--------------------------------------------------------------------------------
//I2C connect check: $ sudo i2cdetect -y 1
//kill deamon $ sudo killall pigpiod
//compile:$ g++ -Wall -pthread -o main Sensor.cpp  main.cpp -lpigpio -lpigpiod_if2 -lrt
//exe active: $ sudo ./main
