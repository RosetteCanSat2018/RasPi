#include <pigpio.h>
#include <iostream>
#include <unistd.h>
#include <stdint.h>
#include "Sensor.h"
#include <stdio.h>

#include "Eigen/Core"
#include "Eigen/LU"
#define _USE_MATH_DEFINES
#include <math.h>
#define PRINT_MAT(X) cout << #X << ":\n" << X << endl << endl
#include "CanSatAD.h"
#include <vector>
#include <fstream>
#include <string>
#include <stdlib.h>

using namespace std;
using namespace Eigen;

Sensor sensor;
CanSatAD AD;

double Mg[3], ACGY[6];
int s1, s2, m1, m2, LoopNum;
float GPSdata[2];

VectorXd x(7);
MatrixXd DCM(3, 3);
const int stay_count = 2000;
const double x_origin = 392.3902350602839;
const double y_origin = -71.12593910596843;
const double z_origin = 268.17914875392034;

FILE *fp;

void data();
void data2();

int main()
{
	LoopNum = 0;
	gpioInitialise();
	sensor.getPi();
	sensor.hmcInit();
	sensor.mpuInit();
	sensor.hmcSetConfigA(0x18); //output rate and measurement configuration
	sensor.hmcSetConfigB(0x20);//Recommended Sensor Field Range 
	sensor.hmcSetMode(0x00);//0x01:single measurement mode(160Hz) 0x00:Continuous measurement mode
	sensor.mpuSetConfig();
	fp = fopen("/home/pi/Sensor/sensorlog.csv", "w");
	//gpioTime(0,&s1,&m1);
	//fprintf(fp,"%d.%03d seconds\r\n",s1,m1/1000);

	AD.SetParameter(x_origin, y_origin, z_origin);

	while (1)
	{
		gpioSetTimerFunc(0, 10, data);
		if (LoopNum >= stay_count)
		{
			break;
		}
	}
	//sensor.GPSGetData(GPSdata);
	//gpioTime(0,&s2,&m2);
	//fprintf(fp,"%d.%03d seconds\r\n",s2,m2/1000);

	AD.OffsetGyro();
	AD.OffsetMagnet();
	AD.PreAnalysis();
	AD.SetInitial(1000);
	AD.Converge(LoopNum);

	int a;
	cin >> a;

	while (1)
	{
		gpioSetTimerFunc(1, 10, data2);
		if (LoopNum >= 8000)
		{
			break;
		}
	}

	fclose(fp);
	sensor.pigpioStop();
}

void data()
{
	sensor.hmcGetXYZ(Mg);
	sensor.mpuGetMotion6(ACGY);
	fprintf(fp, "%lf;%lf;%lf;%lf;%lf;%lf,%lf,%lf,%lf\r\n", ACGY[0], ACGY[1], ACGY[2], ACGY[3], ACGY[4], ACGY[5], Mg[2], -1 * Mg[0], Mg[1]);

	AD.GetValue(LoopNum, ACGY, Mg);

	LoopNum++;
}

void data2()
{
	sensor.hmcGetXYZ(Mg);
	sensor.mpuGetMotion6(ACGY);
	fprintf(fp, "%lf;%lf;%lf;%lf;%lf;%lf,%lf,%lf,%lf\r\n", ACGY[0], ACGY[1], ACGY[2], ACGY[3], ACGY[4], ACGY[5], Mg[2], -1 * Mg[0], Mg[1]);

	AD.GetValue(ACGY, Mg);
	AD.AtittudeEstimate();

	LoopNum++;
}




//How To Use--------------------------------------------------------------------------------
//I2C connect check: $ sudo i2cdetect -y 1
//kill deamon $ sudo killall pigpiod
//compile:$ g++ -Wall -pthread -o main Sensor.cpp  main.cpp -lpigpio -lpigpiod_if2 -lrt
//exe active: $ sudo ./main
