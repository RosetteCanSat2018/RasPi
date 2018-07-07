#ifndef STEPPER_H
#define STEPPER_H

class Stepper
{
public:
	void SetGPIO(int PinA1_, int PinA2_, int PinB1_, int PinB2_);

	void SetWaitTime(double wait);

	void StepCW(int step);

	void StepCCW(int step);

	//void SetPosition(int step, int duration);

private:
	int PinA1, PinA2, PinB1, PinB2;
	double StepWait;
	int Step;

};

#endif /* STEPPER_H */