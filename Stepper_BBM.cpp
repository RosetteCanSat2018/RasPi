#include <pigpio.h>
#include "Stepper.h"

Stepper StepMotor;

int main()
{
	gpioInitialise();
	StepMotor.SetGPIO(4, 17, 27, 22);
	
	StepMotor.StepCW(100);
	StepMotor.StepCCW(100);

	return 0;
}
