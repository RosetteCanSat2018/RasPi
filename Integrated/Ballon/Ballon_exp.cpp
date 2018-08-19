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
//#include "PID.h"

#define HEAT_PERIOD 500 // [msec]
#define HEAT_CYCLE 0 //[msec]
#define HEAT_LOOP 10 //HEAT_PERIOD + HEAT_CYCLE [msec/LOOP]
#define CONTROL_RATE_1 3000// [millisec]
#define ALT_TIMER 3 // [sec] in a row
#define SEA_LEVEL_PRESSURE 1002.8 // [hPa]
#define GPS_SEND_RATE 1 // [sec]
#define LANDING_COUNTER 10

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

//program Ctrl+C-------------------------------------------------------------------------
void escape_program(int sig)
{
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

double alt;
char GPS[2];
MatrixXd DCM(3, 3);

float GPSf[2],GPSf_prev[2];
double alt_prev;
int landing_counter;

// for Servo
int countA = 0;

int main() {
	/* initialise ----------------------------------------------*/
	gpioInitialise();
	bno.Init();
	youdan.Init(16, 20, 4);
	mu.setPin();
	servo.SetServo(19, 26);
	servo.MoveServo(0); //set initial position
	sensor.mplSetConfig(); //mpl configulation
	//pid1.Init(P_GAIN_1, I_GAIN_1, D_GAIN_1);
	//pid2.Init(P_GAIN_2, I_GAIN_2, D_GAIN_2);
	GPS_prev[0] = 0;
	GPS_prev[1] = 0;
	alt_prev = 0;
	gpioSetMode(5, PI_OUTPUT);
	gpioSetMode(22, PI_OUTPUT);
	gpioWrite(5, 0);
	gpioWrite(5, 1);
	gpioWrite(22, 0);

	/* launch ~ expand paddle -----------------------------------*/
	while (1) {
		bno.AttitudeDeterminate();
		if (youdan.DetectOpenParachute()) {
			break;
		}
	}
	for (int i = 0; i < 5; i++) {
		mu.Send("Parachute Open");
	}
	youdan.HeatNichrome(HEAT_PERIOD*1000, HEAT_CYCLE*1000, HEAT_LOOP);

	/* full mission, attitude control using 9-axis sensor ------------*/
	fp = fopen("/home/pi/FM/LogData1.csv", "w");
	while (1) {
		gpioSetTimerFunc(0, CONTROL_RATE_1, func1);

		//landing detect
		if (landing_counter > LANDING_COUNTER) {
			cout << "landing" << endl;
			gpioSetTimerFunc(0, CONTROL_RATE_1, NULL);
			fclose(fp);
			gpioWrite(22, 1);
			break;
		}
		signal(SIGINT, escape_program);
	}

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
	sensor.GPSGetData_f(GPSf);
	DCM = bno.GetDCM();
	/* send gps & landing detect */
	if (LoopNum1 % (int)(GPS_SEND_RATE * 1000 / CONTROL_RATE_1) == 0) {
		mu.Send(GPS[0]);
		mu.Send(GPS[1]);
		mu.Send("-----");

		if (fabsf(GPSf[0] - GPSf_prev[0]) == 0 && fabsf(GPSf[1] - GPSf_prev[1]) == 0 && fabsf(alt_prev - alt) <= 5.0){
			landing_counter++;
		}
		else{
			landing_counter = 0;
		}

		GPSf_prev[0] = GPSf[0];
		GPSf_prev[1] = GPSf[1];
		alt_prev = alt;
		cout << GPSf_0_counter << ";" << GPSf_1_counter << ";" << alt_counter << endl;
		cout << GPSf[0] << ";" << GPSf[1] << ";" << alt << endl;
	}

	/* save data */
	fprintf(fp, "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%s,%s,%f,%f,%f,%f,%f,%f,%f\r\n", accel.x(), accel.y(), accel.z(), gyro.x(), gyro.y(), gyro.z(), magnet.x(), magnet.y(), magnet.z(), alt, GPS[0], GPS[1], DCM(0, 0), DCM(0, 1), DCM(0, 2), DCM(1, 0), DCM(1, 1), DCM(1, 2), DCM(2, 0), DCM(2, 1), DCM(2, 2));
	//cout << accel.x() << "\t" << accel.y() << "\t" << accel.z() << "\t" << gyro.x() << "\t" << gyro.y() << "\t" << gyro.z() << "\t" << magnet.x() << "\t" << magnet.y() << "\t" << magnet.z() << endl;

	if (countA % 4 == 0){
		servo.MoveServo(45);
	} else if (countA % 4 == 1){
		servo.MoveServo(0);
	} else if (countA % 4 == 2){
		servo.MoveServo(-45);
	} else {
		servo.MoveServo(0);
	}
	countA++;
}
