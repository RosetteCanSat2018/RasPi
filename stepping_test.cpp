#include "stepping_move.h"

Stepping motor;

int main()
{
	motor.getPi();
	motor.decidePin();
	motor.rotate360(5000);
}
