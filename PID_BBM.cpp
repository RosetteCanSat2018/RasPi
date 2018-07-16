#include <pigpio.h>
#include <iostream>
#include "PID.h"
#include "Stepper.h"
#include <unistd.h> //for sleep()

using namespace std;

int main()
{
    Stepper StepMotor;
    gpioInitialise();
	StepMotor.SetGPIO(4, 17, 27, 22);

    PID pid;
    pid.Init(0.00001,0.001,0.03); //Input gain

    double error; 
    double set_angle = 0; // target angle
    double angle; // current angle calcurated by Kalmann
    double steer_value; // amount of PID manipulation
    int rotate_step; // rotate step of motor
	int counter = 60;

    while(1){
		if (counter == 360) {
			counter = 60;
		}

		angle = counter;
         cout << "angle=" << angle << endl;

        // calcurate current error
        error = set_angle - angle;
        cout << "error=" << error << endl; 

        // update error
        pid.UpdateError(error);

        // calculate PID manipulation
        steer_value = pid.TotalError();
        cout << "steer_value=" << steer_value <<endl;

        // conversion rotate step
	    rotate_step = int(steer_value / 1.8);

        // move motor
        if (rotate_step > 0){
            StepMotor.StepCW(rotate_step);
        }else if (rotate_step < 0){
            StepMotor.StepCCW(abs(rotate_step));
            cout << "rotate_step = "<< rotate_step << endl;
        }
		sleep(1);
		counter = counter + 60;
    }
}
