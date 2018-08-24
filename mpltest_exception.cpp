#include <pigpio.h>
#include <iostream>
#include <unistd.h>
#include <stdint.h>
#include "Sensor.h"
#include <stdio.h>
#include <signal.h>

#define TIMER 10

Sensor sensor;

using namespace std;

double alt;
int s1, s2, m1, m2, escape;

FILE *fp;

void data();
void escape_program(int sig);

int main()
{
	gpioInitialise();
	sensor.mplSetConfig();

	fp = fopen("/home/pi/BBM/Sensor/sensorlog.csv", "w");
	gpioTime(0, &s1, &m1);
	fprintf(fp, "%d.%03d seconds\r\n", s1, m1 / 1000);
	while (1)
	{
		gpioSetTimerFunc(0, TIMER, data);
		if (s2 - s1 > 50000)
		{
			break;
		}
		signal(SIGINT, escape_program); //Ctrl+C 
	}
	fprintf(fp, "%d.%03d seconds\r\n", s2, m2 / 1000);
	fclose(fp);
	sensor.pigpioStop();
}

void data()
{
	alt = sensor.mplGetALT(1007.0); //hPa(äCñ ãCà≥ÇÃë„ì¸Åj
	cout << alt << endl;
	fprintf(fp, "%f\r\n", alt);
	gpioTime(0, &s2, &m2);
}

//program Ctrl+C-------------------------------------------------------------------------
void escape_program(int sig)
{
	cout << "escape from program" << endl;
	fprintf(fp, "%d.%03d seconds\r\n", s2, m2 / 1000);
	fclose(fp);
	exit(1);
}
