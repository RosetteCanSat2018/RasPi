#include <pigpio.h>
#include <iostream>
#include "Servo.h"
#include <unistd.h>
#include "RPi_Sensor.h"
#include "RPi_BNO055.h"
#include "imumaths.h"
#include "PID.h"

using namespace std;

#define CONTROL_RATE 30 // [millisec]
// for mode 1
#define P_GAIN_1 0.6
#define I_GAIN_1 0
#define D_GAIN_1 0
#define GYRO_CONDITION_UPPER 400 // [rad/sec]
#define ERROR_SATURATION_1 250 // [rad/sec]
#define GYRO_CONDITION_LOWER 100 // [rad/sec]
// for mode 2
#define P_GAIN_2 1
#define I_GAIN_2 0.1
#define D_GAIN_2 0.3
#define ERROR_SATURATION_2 90 // [deg]




Adafruit_BNO055 bno = Adafruit_BNO055();
Servo servo;
PID pid1;
PID pid2;

//imu::Vector<3> gyro;
double gyro_z; // for control mode 1
int mode_flag = 0; // 1:mode1, 0:mode2
double error;
double set_angle = 0; // target angle
double raw_angle; // angle from bno055
double prepreangle = 0; // for median filter
double preangle = 0;
int p_flag = 0;
int n_flag = 0;
double angle; // filtered angle
double totalerror;
int paddledegree; // amount of PID manipulation, rotate step of motor

void func();

double MedianFilter(double val1, double val2, double val3)
{
	if((val2 <= val1 && val1 <= val3) || (val3 <= val1 && val1 <= val2)){
		return val1;
	}else if((val3 <= val2 && val2 <= val1) || (val1 <= val2 && val2 <= val3)){
		return val2;
	}else{
		return val3;
	}
}

int main() {
	gpioInitialise();
	servo.SetServo();
	bno.Init();
	//bno.SerialInit();
	pid1.Init(P_GAIN_1, I_GAIN_1, D_GAIN_1);
	pid2.Init(P_GAIN_2, I_GAIN_2, D_GAIN_2);

	while(1) {
		gpioSetTimerFunc(0, CONTROL_RATE, func);
	}
}


void func() {
	bno.AttitudeDeterminate();

	//cout << bno.GetAngle() << endl;


	//gyro   = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
	//cout << gyro.x() << "\t" << gyro.y() << "\t" << gyro.z() << endl;
	gyro_z = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE).x();
	//cout << gyro_z << endl;

	if (abs(gyro_z) >= GYRO_CONDITION_UPPER) {
		mode_flag = 1;
	} else if (abs(gyro_z) <= GYRO_CONDITION_LOWER) {
		mode_flag = 0;
	}

	if (mode_flag) {
		error = 0 - gyro_z;
		pid1.UpdateError(error);
		totalerror = pid1.TotalError();

		if (error > ERROR_SATURATION_1) {
			paddledegree = -35;
		} else if (error < -ERROR_SATURATION_1) {
			paddledegree = 35;
		} /*else {
			paddledegree
		}*/

		servo.MoveServo(paddledegree);
		cout << "mode 1" << "\t" << gyro_z << "\t" << paddledegree << endl;

		prepreangle = 0;
		preangle = 0;

	} else {
		raw_angle = bno.GetAngle();
		//bno.SerialPrint();
		//raw_angle = MedianFilter(prepreangle, preangle, raw_angle);

		if ((p_flag || n_flag)) {
			// take down the flag
			if (raw_angle > 120) {
				p_flag = 0;
			} else if (raw_angle < -120) {
				n_flag = 0;
			}
		} else {
			// raise the flag
			if (preangle > 150 && raw_angle < 0) {
				p_flag = 1;
				n_flag = 0;
			} else if (preangle < -150 && raw_angle > 0) {
				p_flag = 0;
				n_flag = 1;
			}
		}

		if (p_flag) {
			angle = 179;
		} else if (n_flag) {
			angle = -179;
		} else {
			angle = raw_angle;
		}

		error = set_angle - angle;
		pid2.UpdateError(error);
		totalerror = pid2.TotalError();

		if (error > ERROR_SATURATION_2) {
			paddledegree = -35;
		} else if (error < -ERROR_SATURATION_2) {
			paddledegree = 35;
		} else {
			paddledegree = (int)totalerror;
		}

		servo.MoveServo(paddledegree);
		cout << "mode 2" << "\t" << (int)raw_angle << "\t" << paddledegree << "\t" << p_flag << "\t" << n_flag << endl;

		prepreangle = preangle;
		preangle = raw_angle;

	}

}
