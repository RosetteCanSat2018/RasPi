#include <pigpio.h>
#include <iostream>
#include <unistd.h>
#include <stdint.h>
#include "Sensor.h"
#include <stdio.h>

<<<<<<< HEAD
using namespace std;
=======
#include "Eigen/Core"
#include "Eigen/LU"
#define _USE_MATH_DEFINES
#include <math.h>
#define PRINT_MAT(X) cout << #X << ":\n" << X << endl << endl
#include "CanSatAD.h"
>>>>>>> f53f1a108328fec1fa54e8f73c650faadcc18f89

Sensor sensor;
CanSatAD AD;

<<<<<<< HEAD
double Mg[3],ACGY[6];
int s1,s2,m1,m2,i;
float GPSdata[2];
=======
using namespace std;
using namespace Eigen;

double Sdata[9];
>>>>>>> f53f1a108328fec1fa54e8f73c650faadcc18f89

VectorXf x(7);
MatrixXf DCM(3, 3);

FILE *fp;

void data();
<<<<<<< HEAD

int main()
{
	i=0;
=======
void data2();
int LoopNum = 0;
int main()
{
	int s1, s2, m1, m2;
>>>>>>> f53f1a108328fec1fa54e8f73c650faadcc18f89
	gpioInitialise();
	sensor.getPi();
	sensor.hmcInit();
	sensor.mpuInit();
	sensor.hmcSetConfigA(0x18); //output rate and measurement configuration
	sensor.hmcSetConfigB(0x20);//Recommended Sensor Field Range 
	sensor.hmcSetMode(0x00);//0x01:single measurement mode(160Hz) 0x00:Continuous measurement mode
	sensor.mpuSetConfig();
<<<<<<< HEAD
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
=======
	fp = fopen("/home/pi/sensorlog1.csv", "w");
	gpioTime(0, &s1, &m1);
	fprintf(fp, "%d.%03d seconds\r\n", s1, m1 / 1000);

	AD.SetParameter(10, 10, 10);

	while (1)
	{
		gpioSetTimerFunc(0, 10, data);
		if (LoopNum >= 1500)
		{
			break;
		}
	}

	AD.OffsetGyro();
	AD.OffsetMagnet();
	AD.PreAnalysis();
	AD.SetInitial(1000);
	AD.Converge(LoopNum);

	while (1)
	{
		gpioSetTimerFunc(1, 10, data2);
	}
	gpioTime(0, &s2, &m2);
	fprintf(fp, "%d.%03d seconds\r\n", s2, m2 / 1000);
>>>>>>> f53f1a108328fec1fa54e8f73c650faadcc18f89
	fclose(fp);
	sensor.pigpioStop();
}

void data()
{
<<<<<<< HEAD
	sensor.hmcGetXYZ(Mg);
	sensor.mpuGetMotion6(ACGY);
	fprintf(fp,"%lf;%lf;%lf;%lf;%lf;%lf,%lf,%lf,%lf\r\n",Mg[2],-1*Mg[0],Mg[1],ACGY[0],ACGY[1],ACGY[2],ACGY[3],ACGY[4],ACGY[5]);
	i++;
}


=======
	sensor.GetMotion9(Sdata);
	fprintf(fp, "%f;%f;%f;%f;%f;%f,%f,%f,%f\r\n", Sdata[0], Sdata[1], Sdata[2], Sdata[3], Sdata[4], Sdata[5], Sdata[6], Sdata[8], Sdata[7]);

	AD.GetValue(LoopNum, Sdata);

	LoopNum++;
}

void data2()
{
	sensor.GetMotion9(Sdata);
	fprintf(fp, "%f;%f;%f;%f;%f;%f,%f,%f,%f\r\n", Sdata[0], Sdata[1], Sdata[2], Sdata[3], Sdata[4], Sdata[5], Sdata[6], Sdata[8], Sdata[7]);

	AD.GetValue(Sdata);
	AD.AtittudeEstimate();
	x = AD.GetStateVariable();
	DCM = AD.GetDCM();

}
>>>>>>> f53f1a108328fec1fa54e8f73c650faadcc18f89

//How To Use--------------------------------------------------------------------------------
//I2C connect check: $ sudo i2cdetect -y 1
//kill deamon $ sudo killall pigpiod
//compile:$ g++ -Wall -pthread -o main Sensor.cpp  main.cpp -lpigpio -lpigpiod_if2 -lrt
//exe active: $ sudo ./main
