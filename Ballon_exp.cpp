#include <pigpio.h>
#include <iostream>
#include <unistd.h>
#include <stdint.h>
#include <stdio.h>
#include "Eigen/Core"
#include "Eigen/LU"
#define _USE_MATH_DEFINES
#include <math.h>

#include "BNO055/RPi_Sensor.h"
#include "BNO055/RPi_BNO055.h"
#include "BNO055/imumaths.h"
#include "Youdan/Youdan.h"
#include "Sensor/Sensor.h"
#include "Servo/Servo.h"
//#include "PID/PID.h"

#define P_GAIN_1 0.3
#define I_GAIN_1 0
#define D_GAIN_1 0
#define P_GAIN_2 0.3
#define I_GAIN_2 0
#define D_GAIN_2 0
#define HEAT_PERIOD 500 // [msec] 
#define HEAT_CYCLE 500 //[msec]
#define HEAT_LOOP 5 //HEAT_PERIOD + HEAT_CYCLE [msec/LOOP]
#define CONTROL_RATE_1 10 // [millisec]
#define CONTROL_RATE_2 10 // [millisec]
#define ALT_CONDITION 800 // [m]
#define ALT_TIMER 3 // [sec] in a row
#define SEA_LEVEL_PRESSURE 1002.8 // [hPa]
#define GPS_SEND_RATE 1 // [sec]
#define GYRO_CONDITION 1 // [rad/sec]
#define ERROR_CONDITION 3 // [degree]
#define FULL_TIMER 10 // [sec] in a row

#define END_SECOND 

using namespace std;
using namespace Eigen;

Adafruit_BNO055 bno = Adafruit_BNO055();
Youdan youdan;
Sensor sensor;
//MU2 mu;
Servo servo;
//PID pid1;
//PID pid2;

FILE *fp1;
//FILE *fp2;

void func1();
//void func2();

// for attitude condition
/*
int count1 = 0;
bool alt_condition;
*/
// for full success condition
/*
int count2 = 0;
bool full_success;
*/
// for landing alt_condition
/*
int count3 = 0;
bool landing_condition;
*/
// for ballon experimence
int START_SECOND, START_MSECOND, FUNC1_SECOND, FUNC1_MSECOND;//[sec],[msec]

// - VECTOR_ACCELEROMETER - m/s^2
// - VECTOR_MAGNETOMETER  - uT
// - VECTOR_GYROSCOPE     - rad/s
// - VECTOR_EULER         - degrees
// - VECTOR_LINEARACCEL   - m/s^2
// - VECTOR_GRAVITY       - m/s^2
imu::Vector<3> accel;
imu::Vector<3> gyro;
imu::Vector<3> magnet;
double alt;
float GPS[2];
MatrixXd DCM(3, 3);

int main() {
	/* initialise ----------------------------------------------*/
	gpioInitialise();
	bno.Init();
	youdan.Init(23, 24, 25);
	//mu.setPin();
	servo.SetServo();
	servo.MoveServo(0); //set initial position
	sensor.mplSetConfig(); //mpl configulation
	//pid1.Init(P_GAIN_1, I_GAIN_1, D_GAIN_1);
	//pid2.Init(P_GAIN_2, I_GAIN_2, D_GAIN_2);

	/*measure ballon experimence's starttime */
	gpioTime(0, &START_SECOND, &START_MSECOND);

	/* launch ~ expand paddle -----------------------------------*/
	while (1) {
		bno.AttitudeDeterminate();
		if (youdan.DetectOpenParachute()) {
			break;
		}
	}
	/*
	for (int i = 0; i < 5; i++) {
		mu.Send("Parachute Open");
	}
	*/
	youdan.HeatNichrome(HEAT_PERIOD*1000, HEAT_CYCLE*1000, HEAT_LOOP);


	/* full mission, attitude control using 9-axis sensor ------------*/
	fp1 = fopen("/home/pi/FM/LogData1.csv", "w");
	while (1) {
		/***** 周期を調べる必要あり ***************/
		gpioSetTimerFunc(0, CONTROL_RATE_1, func1);
		/*
		if (alt_condition && full_success) {
			fclose(fp1);
			gpioSetTimerFunc(0, CONTROL_RATE_1, NULL);
			break;
		}

		if (LandingCondition()) {
			fclose(fp1);
			gpioSetTimerFunc(0, CONTROL_RATE_1, NULL);
			break;
		}
		*/
		/*for ballon experimence*/
		if (FUNC1_SECOND - START_SECOND > END_SECOND)
		{
			fclose(fp1);
			gpioSetTimerFunc(0, CONTROL_RATE_1, NULL);
			break;
		}
	}

	/* advanced mission, attitude control using camera ------------------*/
	/*
	fp2 = fopen("/home/pi/FM/LogData2.csv", "w");
	while (1) {
		gpioSetTimerFunc(1, CONTROL_RATE_2, func2);

		if (LandingCondition()) {
			fclose(fp2);
			gpioSetTimerFunc(1, CONTROL_RATE_2, NULL);
			break;
		}
	}
	*/
	
}



void func1()
{
	bno.AttitudeDeterminate();

	/* get sensor value */
	accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
	gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
	magnet = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
	alt = sensor.mplGetALT(SEA_LEVEL_PRESSURE);
	sensor.GPSGetData(GPS);
	DCM = bno.GetDCM();

	/* send gps */
	/*******************GPSの送信方法わからん*******************/
	/*
	if (LoopNum1 % (int)(GPS_SEND_RATE * 1000 / CONTROL_RATE_1) == 0) {
		mu.Send();
	}
	*/
	/* save data */
	//fprintf(fp1, "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\r\n", accel.x(), accel.y(), accel.z(), gyro.x(), gyro.y(), gyro.z(), magnet.x(), magnet.y(), magnet.z(), alt, GPS[0], GPS[1], DCM(0, 0), DCM(0, 1), DCM(0, 2), DCM(1, 0), DCM(1, 1), DCM(1, 2), DCM(2, 0), DCM(2, 1), DCM(2, 2));
	//fprintf(fp1, "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\r\n", accel.x(), accel.y(), accel.z(), gyro.x(), gyro.y(), gyro.z(), magnet.x(), magnet.y(), magnet.z(), alt, GPS[0], GPS[1], DCM(0, 0), DCM(0, 1), DCM(0, 2), DCM(1, 0), DCM(1, 1), DCM(1, 2), DCM(2, 0), DCM(2, 1), DCM(2, 2));
	cout << accel.x() << "\t" << accel.y() << "\t" << accel.z() << "\t" << gyro.x() << "\t" << gyro.y() << "\t" << gyro.z() << "\t" << magnet.x() << "\t" << magnet.y() << "\t" << magnet.z() << endl;
	/* attitude control */

	/* attitude condition for proceeding to the advanced mission */
	/*
	if (alt < ALT_CONDITION) {
		count1++;
	}
	else {
		count1 = 0;
	}
	if (count1 == ((int)(ALT_TIMER * 1000 / CONTROL_RATE_1))) {
		alt_condition = true;
	}
	else {
		alt_condition = false;
	}
	*/
	/* full success, clear or not */
	/*
	if (error < ERROR_CONDITION) {
		count2++;
	}
	else {
		count2 = 0;
	}
	if (count2 == ((int)(FULL_TIMER * 1000 / CONTROL_RATE_2))) {
		full_success = true;
	}
	else {
		full_success = false;
	}
	*/
	gpioTime(0, &FUNC1_SECOND, &FUNC1_MSECOND);

}

void func2() {

}
