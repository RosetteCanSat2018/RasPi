#include <pigpio.h>
#include <iostream>
#include <unistd.h>
#include <stdint.h>
#include <stdio.h>
#include "Eigen/Core"
#include "Eigen/LU"
#define _USE_MATH_DEFINES
#include <math.h>
#include <signal.h>
#include "RPi_Sensor.h"
#include "RPi_BNO055.h"
#include "imumaths.h"
#include "Youdan.h"
#include "Sensor.h"
#include "Servo.h"
#include <gps.h>
#include "MU2.h"
//#include "PID.h"

#define HEAT_PERIOD 1 // [msec]
#define HEAT_CYCLE 130 //[msec]
#define HEAT_LOOP 50 //50 //HEAT_PERIOD + HEAT_CYCLE [msec/LOOP]
#define CONTROL_RATE_1 10// [millisec]
#define SEA_LEVEL_PRESSURE 1008.4 // [hPa]
#define GPS_SEND_RATE 20 // [sec]
#define MISSION_TIME 1200 // [sec]

using namespace std;
using namespace Eigen;

Adafruit_BNO055 bno = Adafruit_BNO055();
Youdan youdan;
Sensor sensor;
MU2 mu;
Servo servo;
//PID pid1;
//PID pid2;
FILE *fp;

int s0, m0;
int s1 = 0;
int m1 = 0;
int s2, m2, i;
double freq;

//program Ctrl+C-------------------------------------------------------------------------
void escape_heat(int sig) {
	cout << "ctrl+c" << endl;
	gpioWrite(4, 0);
	exit(1);
}
void escape_program(int sig) {
	cout << "ctrl+c" << endl;
	fclose(fp);
	gpioSetTimerFunc(0, CONTROL_RATE_1, NULL);
	exit(1);
}

void func1();

// - VECTOR_ACCELEROMETER - m/s^2
// - VECTOR_MAGNETOMETER  - uT
// - VECTOR_GYROSCOPE     - rad/s
// - VECTOR_EULER         - degrees
// - VECTOR_LINEARACCEL   - m/s^2
// - VECTOR_GRAVITY       - m/s^2
imu::Vector<3> accel;
imu::Vector<3> gyro;
imu::Vector<3> magnet;
MatrixXd DCM(3, 3);

float GPS[2];
double alt;

int LoopNum1 = 0;

int paddledegree = 0;

/* for Ballon test */
int paddlemode = 0;

int main() {
	/* initialise ----------------------------------------------*/
	gpioInitialise();
	bno.Init();
	youdan.Init(16, 20, 4);
	servo.SetServo(19, 26);
	servo.MoveServo(0); //set initial position
	sensor.mplSetConfig(); //mpl configulation
	//pid1.Init(P_GAIN_1, I_GAIN_1, D_GAIN_1);
	//pid2.Init(P_GAIN_2, I_GAIN_2, D_GAIN_2);
	gpioSetMode(5, PI_OUTPUT);
	gpioSetMode(22, PI_OUTPUT);
	gpioWrite(5, 0);
	gpioWrite(5, 1); // Red LED ON, indicate program start correctly
	gpioWrite(22, 0); // Green LED OFF

	/* launch ~ expand paddle -----------------------------------*/
	while (1) {
		bno.AttitudeDeterminate();
		if (youdan.DetectOpenParachute()) {
			gpioTime(0, &s0, &m0);
			break;
		}
	}
	/*
	for (int i = 0; i < 3; i++) {
		mu.Send("Parachute Open");
		sleep(1);
	}
	*/

	for(int i=0;i<HEAT_LOOP;i++) {
		youdan.HeatNichrome(HEAT_PERIOD*1000, HEAT_CYCLE*1000);
		signal(SIGINT,escape_heat);
	}
	//while(1){signal(SIGINT,escape_heat);}

	/* full mission, attitude control using 9-axis sensor ------------*/
	fp = fopen("/home/pi/Ballon/LogDataFM.csv", "w");
	while (1) {
		gpioSetTimerFunc(0, CONTROL_RATE_1, func1);

		if ((s2 - s0) > MISSION_TIME) {
			cout << "20 minutes" << endl;
			gpioSetTimerFunc(0, CONTROL_RATE_1, NULL);
			fclose(fp);
			gpioWrite(22, 1); // Green LED ON
			break;
		}
		signal(SIGINT, escape_program);

		// in case collected in MISSION_TIME[sec]
		if (gpioRead(16) == 0){
			fclose(fp);
			gpioSetTimerFunc(0, CONTROL_RATE_1, NULL);
			cout << "flight pin on" << endl;
			break;
		}
	}
}


void func1() {
	gpioTime(0,&s2,&m2);
	freq = (s2+(double)m2/1000000)-(s1+(double)m1/1000000);
	cout << freq << " seconds" << endl << endl;
	s1 = s2;
	m1 = m2;

	bno.AttitudeDeterminate();
	/* get sensor value */
	accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
	gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
	magnet = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
	alt = sensor.mplGetALT(SEA_LEVEL_PRESSURE);
	DCM = bno.GetDCM();

	/* send gps */
	if (LoopNum1 % (int)(GPS_SEND_RATE * 1000 / CONTROL_RATE_1) == 0) {
		sensor.GPSGetData_f(GPS);
		mu.SendGPS(GPS[0]);
		mu.SendGPS(GPS[1]);
		//cout << GPS[0] << ";" << GPS[1] << ";" << alt << endl;
	}
	LoopNum1++;


	/* paddle move */
	if (LoopNum1 % 2 == 0) {
		servo.MoveServo(paddledegree);
		if (paddledegree == 60) {
			paddlemode = 1;
		} else if (paddledegree == -60) {
			paddlemode = 0;
		}

		if (!paddlemode) {
			paddledegree++;
		} else {
			paddledegree--;
		}
	}

	/* save data */
	fprintf(fp, "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,\r\n", freq, accel.x(), accel.y(), accel.z(), gyro.x(), gyro.y(), gyro.z(), magnet.x(), magnet.y(), magnet.z(), alt, GPS[0], GPS[1], DCM(0, 0), DCM(0, 1), DCM(0, 2), DCM(1, 0), DCM(1, 1), DCM(1, 2), DCM(2, 0), DCM(2, 1), DCM(2, 2), paddledegree);
}
