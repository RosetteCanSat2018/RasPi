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

	while(1) {
		servo.MoveServo(0, 0);
		sleep(2);
		servo.MoveServo(90, 90);
		sleep(2);
		servo.MoveServo(-90, -90);
		sleep(2);
	}
}
