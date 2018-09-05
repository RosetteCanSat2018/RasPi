#include <pigpio.h>
#include <iostream>
#include "Servo.h"
#include <unistd.h>
#include "RPi_Sensor.h"
#include "RPi_BNO055.h"
#include "imumaths.h"
#include "PID.h"
#include "Youdan.h"
#include <iomanip>

using namespace std;

#define CONTROL_RATE 10.0 // [millisec]
#define SERVO_OFFSET_1 9
#define SERVO_OFFSET_2 1

// for mode switch
#define MODE_CONDITION_UPPER 120.0 // [deg/sec]
#define MODE_CONDITION_LOWER 20.0 // [deg/sec]
#define MODE_PERIOD_CONDITION 2.0 // [sec], mode0 >> mode1

/* for mode 0, PD control */
#define P_GAIN_1 50.0
#define I_GAIN_1 0
#define D_GAIN_1 70.0
#define SATURATION_1 5000.0
#define MAX_PADDLE_DEGREE_1 20.0

/* for mode 1, PI control */
#define P_GAIN_2 500.0
#define I_GAIN_2 0.3
#define D_GAIN_2 100000
#define SATURATION_2 100000.0
#define MAX_PADDLE_DEGREE_2 7.0

/* for success criteria */
#define FULL_DEGREE_CONDITION 30.0
#define ADVANCED_DEGREE_CONDITION 10.0
#define PERIOD_CONDITION 10.0

#define FULL_ACHIEVE_TIMES 2 //full to advanced

Adafruit_BNO055 bno = Adafruit_BNO055();
Servo servo;
PID pid1;
PID pid2;
Youdan youdan;

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

int full_i_error_reset_count = 0;
int advanced_i_error_reset_count = 0;

int mode_count = 0;

int s1 = 0;
int m1 = 0;
int s2, m2, i;
double freq;

int full_success = 0;
int advanced_success = 0;


FILE *fp;

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
	youdan.Init(16, 20, 4);
	servo.SetServo(19, 26);
	servo.MoveServo(0+SERVO_OFFSET_1, 0+SERVO_OFFSET_2);
	//while(1){}
	bno.Init();
	//bno.SerialInit();
	pid1.Init(P_GAIN_1, I_GAIN_1, D_GAIN_1);
	pid2.Init(P_GAIN_2, I_GAIN_2, D_GAIN_2);

	while (1) {
		bno.AttitudeDeterminate();
		if (youdan.DetectOpenParachute()) {
			break;
		}
	}

	//fp = fopen("/home/pi/ADC/LogDataBallon.csv", "w");

	while(1) {
		gpioSetTimerFunc(0, CONTROL_RATE, func);

		// in case collected in MISSION_TIME[sec]
		if (gpioRead(16) == 0){
			fclose(fp);
			gpioSetTimerFunc(0, CONTROL_RATE, NULL);
			cout << "flight pin on" << endl;
			break;
		}
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
		mode_count = 0;
	} else if (abs(gyro_z) < MODE_CONDITION_LOWER) {
		mode_count++;
		if (mode_count > (MODE_PERIOD_CONDITION/(CONTROL_RATE/1000))) {
			mode = 1;
			p_flag = 0;
			n_flag = 0;
			mode_count = 0;
		}
	} else {
		mode_count = 0;
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


		if (-FULL_DEGREE_CONDITION < error && error < FULL_DEGREE_CONDITION) {
	  	  full_i_error_reset_count++;
	  	  if (full_i_error_reset_count > (PERIOD_CONDITION/(CONTROL_RATE/1000))) {
			  full_i_error_reset_count = 0;
			  full_success++;
	  	  }
	    } else{
  	  		  full_i_error_reset_count = 0;
	    }

		if (full_success < FULL_ACHIEVE_TIMES && -FULL_DEGREE_CONDITION < error && error < FULL_DEGREE_CONDITION && !(-ADVANCED_DEGREE_CONDITION < error && error < ADVANCED_DEGREE_CONDITION)) {
			pid2.IerrorReset();
		}

		if (-ADVANCED_DEGREE_CONDITION < error && error < ADVANCED_DEGREE_CONDITION) {
	  	  advanced_i_error_reset_count++;
	  	  if (advanced_i_error_reset_count > (PERIOD_CONDITION/(CONTROL_RATE/1000))) {
	  		  //pid2.IerrorReset();
	  		  advanced_i_error_reset_count = 0;
			  advanced_success++;
	  	  }
	    } else {
	  	    advanced_i_error_reset_count = 0;
	    }

		if (totalerror > SATURATION_2) {
			paddledegree = -MAX_PADDLE_DEGREE_2;
		} else if (totalerror < -SATURATION_2) {
			paddledegree = MAX_PADDLE_DEGREE_2;
		} else {
			paddledegree = totalerror * (MAX_PADDLE_DEGREE_2 / SATURATION_2);
		}

		cout.setf(ios::left, ios::adjustfield);
		cout << setw(8) << preangle << "\t" << setw(8) << raw_angle << "\t" << setw(8) << angle << "\t" << setw(8) <<  totalerror << "\t" << setw(5) << paddledegree << "\t" << setw(5) <<  full_i_error_reset_count << "\t" << advanced_i_error_reset_count << "\t" << mode << "\t" << full_success << "\t" << advanced_success << endl;

		servo.MoveServo(paddledegree + SERVO_OFFSET_1, paddledegree + SERVO_OFFSET_2);
	}

	pregyro = gyro_z;
	preangle = raw_angle;

	//fprintf(fp, "%f,%f,%f,%f,%d,%f,%d,%d,%d,%d \r\n", freq, raw_angle, angle, gyro_z, mode, paddledegree, full_i_error_reset_count, advanced_i_error_reset_count, full_success, advanced_success);

}
