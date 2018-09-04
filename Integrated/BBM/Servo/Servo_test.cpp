#include <pigpio.h>
#include <iostream>
#include "Servo.h"
#include <unistd.h>

Servo servo;
using namespace std;

int main()
{
	gpioInitialise();
	servo.SetServo(19, 26);
	for (int i = 0; i < 10; i++)
	{
		servo.MoveServo(0);
		cout << "deg is 0" << endl;
		sleep(2);
   
		servo.MoveServo(45);
		cout << "deg is 45" << endl;
		sleep(2);
		servo.MoveServo(-45);
		cout << "deg is -45" << endl;
		sleep(2);
   
	}
	cout << "end_test" << endl;
}


