#include "pigpio.h"
#include "Youdan.h"

Youdan youdan;

int main()
{
	gpioInitialise();
	youdan.Init(23, 24, 25);
	youdan.DetectOpenParachute();
	youdan.HeatNichrome(10); // [sec]
}
