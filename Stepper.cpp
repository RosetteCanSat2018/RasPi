#include <pigpio.h>
#include <unistd.h>

#include "Stepper.h"

#define _USE_MATH_DEFINES
#include <math.h>

void Stepper::SetGPIO(int PinA1_, int PinA2_, int PinB1_, int PinB2_)
{
	PinA1 = PinA1_;
	PinA2 = PinA2_;
	PinB1 = PinB1_;
	PinB2 = PinB2_;
	Step = 0;
	
	SetWaitTime(0.01);

	gpioSetMode(PinA1, 1);
	gpioSetMode(PinA2, 1);
	gpioSetMode(PinB1, 1);
	gpioSetMode(PinB2, 1);

	gpioWrite(PinA2, 1);
	gpioWrite(PinB2, 1);
	
	gpioWrite(PinA1, 0);
	gpioWrite(PinB1, 0);

	
}

// ウェイト時間を設定する
void Stepper::SetWaitTime(double wait)
{
	if (wait < 0.01) { StepWait = 0.005; }
	else if (wait > 0.5) { StepWait = 0.1; }
	else { StepWait = wait; }

}

// CWにstepステップ(1.8°* step)移動する
void Stepper::StepCW(int step)
{
	for (int i = 0; i < step; i++)
	{
		gpioWrite(PinA1, 1);
		usleep(StepWait*1000000);
		gpioWrite(PinB1, 1);
		usleep(StepWait*1000000);
		gpioWrite(PinA1, 0);
		usleep(StepWait*1000000);
		gpioWrite(PinB1, 0);
		usleep(StepWait*1000000);
	}
}

// CCWにstepステップ(1.8°* step)移動する
void Stepper::StepCCW(int step)
{
	for (int i = 0; i < step; i++)
	{
		gpioWrite(PinB1, 1);
		usleep(StepWait*1000000);
		gpioWrite(PinA1, 1);
		usleep(StepWait*1000000);
		gpioWrite(PinB1, 0);
		usleep(StepWait*1000000);
		gpioWrite(PinA1, 0);
		usleep(StepWait*1000000);
	}
}


/*
// 目標ポジションに移動する
void Stepper::SetPosition(int step, int duration)
{
	int diff_step = step - Step;
	if (diff_step != 0)
	{
		double wait = abs(duration/diff_step/4);
		SetWaitTime(wait);
	}
	for (int i = 0; i > abs(diff_step); i++)
	{
		if (diff_step > 0){StepCW();}
		if (diff_step < 0){StepCCW();}
	}
	Step = step;
}
*/