#include <pigpio.h>
#include <iostream>
#include <unistd.h>
#include <stdint.h>
#include "Sensor.h"
#include <stdio.h>
#include "PID.h"
#include "Stepper.h"
#include <unistd.h> //for sleep()
#include <math.h>

using namespace std;


Stepper StepMotor;
Sensor sensor;
PID pid;

int main()
{
	gpioInitialise();
	sensor.getPi();
	StepMotor.SetGPIO(4, 17, 27, 22);


	pid.Init(0.04, 0, 0); //Input gain

	double error;
	double set_angle; // target angle
	double angle; // current angle calcurated by Kalmann
	double steer_value; // amount of PID manipulation
	int rotate_step; // rotate step of motor
	int counter=0;


	double ACGY[6];
	double first[6];

	FILE *fp;

	void data();
	sensor.mpuInit();
	sensor.mpuSetConfig();
	fp = fopen("/home/pi/PID_sensorlog.csv", "w");

	sensor.mpuGetMotion6(first);
	first[3] -= 0.326657;
	first[4] -= 0.00771948;
	first[5] -= -0.0171353;
	//set_angle = first[5];
        set_angle = 0;
        cout << "set_angle=" << set_angle << endl;

	while (1) {
		sensor.mpuGetMotion6(ACGY);
		fprintf(fp, "%f;%f;%f,%f,%f,%f\r\n",  ACGY[0], ACGY[1], ACGY[2], ACGY[3], ACGY[4], ACGY[5]);
		printf("%f;%f;%f,%f,%f,%f\r\n",ACGY[0],ACGY[1],ACGY[2],ACGY[3],ACGY[4],ACGY[5]);
		ACGY[3] -= 0.326657;
		ACGY[4] -= 0.00771948;
		ACGY[5] -= -0.0171353;

		angle = ACGY[5];
		cout << "angle=" << angle << endl;

		// calcurate current error
		error = set_angle - angle;
	        cout << "error=" << error << endl;

		// update error
		pid.UpdateError(error);

		// calculate PID manipulation
		steer_value = pid.TotalError();
		cout << "steer_value=" << steer_value << endl;

		// conversion rotate step
		rotate_step = int(steer_value / 1.8);

		// move motor
		if (rotate_step > 0) {
			StepMotor.StepCW(rotate_step);
			cout << "rotate_step = " << rotate_step << "******************* " << endl;
		}
		else if (rotate_step < 0) {
			StepMotor.StepCCW(abs(rotate_step));
			cout << "rotate_step = " << rotate_step << endl;
		}
		usleep(80000);
		counter++;
		if(counter>600){
			break;
		}
	}
	fclose(fp);
}
