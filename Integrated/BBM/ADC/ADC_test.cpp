#include <pigpio.h>
#include <iostream>
#include "Servo.h"
#include <unistd.h>
#include "RPi_Sensor.h"
#include "RPi_BNO055.h"
#include "imumaths.h"
#include "PID.h"
#include <iomanip>

using namespace std;

#define CONTROL_RATE 10.0 // [millisec]
#define SERVO_OFFSET_1 6
#define SERVO_OFFSET_2 1

// for mode switch
#define MODE_CONDITION_UPPER 150.0 // [deg/sec]
#define MODE_CONDITION_LOWER 20.0 // [deg/sec]

/* for mode 0, PD control */
#define P_GAIN_1 50.0
#define I_GAIN_1 0
#define D_GAIN_1 70.0
#define SATURATION_1 5000.0
#define MAX_PADDLE_DEGREE_1 35.0

/* for mode 1, PI control */
#define P_GAIN_2 50.0
#define I_GAIN_2 1.0
#define D_GAIN_2 70.0
#define SATURATION_2 10000.0
#define MAX_PADDLE_DEGREE_2 10.0

#define DEGREE_CONDITION 15.0
#define PERIOD_CONDITION 10.0


Adafruit_BNO055 bno = Adafruit_BNO055();
Servo servo;
PID pid1;
PID pid2;

imu::Vector<3> accel;
imu::Vector<3> gyro;
imu::Vector<3> magnet;
double gyro_z;
double pregyro = 0;
double error;
double set_angle = 0; // target angle
double raw_angle; // angle from bno055
//double prepreangle = 0; // for median filter
double preangle = 0;
int p_flag = 0;
int n_flag = 0;
double angle; // filtered angle
double totalerror;
double paddledegree; // amount of PID manipulation, rotate step of motor

int mode = 0; // 0:PD, 1:PI

int noise_count = 0;
int calibration_count = 0;
int noise_flag = 0;

int i_error_reset_count = 0;

int s1 = 0;
int m1 = 0;
int s2, m2, i;
double freq;

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
	servo.SetServo(19, 26);
	servo.MoveServo(0+SERVO_OFFSET_1, 0+SERVO_OFFSET_2);
	//while(1){}
	bno.Init();
	//bno.SerialInit();
	pid1.Init(P_GAIN_1, I_GAIN_1, D_GAIN_1);
	pid2.Init(P_GAIN_2, I_GAIN_2, D_GAIN_2);

	while(1) {
		gpioSetTimerFunc(0, CONTROL_RATE, func);
	}
}


void func() {

	gpioTime(0,&s2,&m2);
	freq = (s2+(double)m2/1000000)-(s1+(double)m1/1000000);
	//cout << freq << " seconds" << endl << endl;
	s1 = s2;
	m1 = m2;

	/* get value */
	bno.AttitudeDeterminate();
	raw_angle = bno.GetAngle();
	gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
	gyro_z = -gyro.x() *180/M_PI; // [deg/sec]

	/* noise filter */
	if ((1000/CONTROL_RATE*0.5) < calibration_count) {
		if (50 < fabs(raw_angle - preangle) && fabs(raw_angle - preangle) < 330) {
			cout << "ON" << endl;
			noise_count++;
			if (noise_count < 3) {
				raw_angle = preangle;
			} else {
				noise_count = 0;
				cout << endl << endl << endl;
				noise_flag = 1;
			}
		} else {
			noise_count = 0;
		}

		if (1000 < fabs(gyro_z - pregyro)) {
			gyro_z = pregyro;
		}
	}
	calibration_count++;

	/* mode switch */
	if (abs(gyro_z) > MODE_CONDITION_UPPER) {
		mode = 0;
		pid2.IerrorReset();
	} else if (abs(gyro_z) < MODE_CONDITION_LOWER) {
		mode = 1;
	}

	/* control */
	if (!mode) {
		error = 0 - gyro_z;
		pid1.UpdateError(error);
		totalerror = pid1.TotalError();

		if (totalerror > SATURATION_1) {
			paddledegree = MAX_PADDLE_DEGREE_1;
		} else if (totalerror < -SATURATION_1) {
			paddledegree = -MAX_PADDLE_DEGREE_1;
		} else {
			paddledegree = totalerror * (MAX_PADDLE_DEGREE_1 / SATURATION_1);
		}

		cout.setf(ios::left, ios::adjustfield);
		cout << setw(8) << gyro_z << "\t" << setw(8) <<  totalerror << "\t" << setw(5) << (int)paddledegree << "\t" << mode << endl;

		servo.MoveServo((int)paddledegree + SERVO_OFFSET_1, (int)paddledegree + SERVO_OFFSET_2);

	} else {

		if (!noise_flag) {
			if ((p_flag || n_flag)) {
				// take down the flag
				if (raw_angle > 120) {
					p_flag = 0;
				} else if (raw_angle < -120) {
					n_flag = 0;
				}
			} else {
				// raise the flag
				if (preangle > 120 && raw_angle < 0) {
					p_flag = 1;
					n_flag = 0;
				} else if (preangle < -120 && raw_angle > 0) {
					p_flag = 0;
					n_flag = 1;
				}
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

		if (-DEGREE_CONDITION < error && error < DEGREE_CONDITION) {
	  	  i_error_reset_count++;
	  	  if (i_error_reset_count > (PERIOD_CONDITION/(CONTROL_RATE/1000))) {
	  		  pid2.IerrorReset();
	  		  i_error_reset_count = 0;
	  	  }
	    } else {
	  	  i_error_reset_count = 0;
	    }

		if (totalerror > SATURATION_2) {
			paddledegree = -MAX_PADDLE_DEGREE_2;
		} else if (totalerror < -SATURATION_2) {
			paddledegree = MAX_PADDLE_DEGREE_2;
		} else {
			paddledegree = totalerror * (MAX_PADDLE_DEGREE_2 / SATURATION_2);
		}

		cout.setf(ios::left, ios::adjustfield);
		cout << setw(8) << preangle << "\t" << setw(8) << raw_angle << "\t" << setw(8) << angle << "\t" << setw(8) <<  totalerror << "\t" << setw(5) << (int)paddledegree << "\t" << setw(5) <<  i_error_reset_count << "\t" << mode << endl;

		servo.MoveServo((int)paddledegree + SERVO_OFFSET_1, (int)paddledegree + SERVO_OFFSET_2);
	}

	pregyro = gyro_z;
	preangle = raw_angle;

}
