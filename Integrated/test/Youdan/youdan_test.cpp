#include "pigpio.h"
#include "Youdan.h"
#include <signal.h>
#include <iostream>


#define HEAT_PERIOD 1 // [msec]
#define HEAT_CYCLE 130 //[msec]
#define HEAT_LOOP 50 //50 //HEAT_PERIOD + HEAT_CYCLE [msec/LOOP]

using namespace std;

Youdan youdan;

void escape_heat(int sig) {
	cout << "ctrl+c" << endl;
	gpioWrite(4, 0);
	exit(1);
}

int main()
{
	gpioInitialise();
	youdan.Init(16, 20, 4);

	while (1) {
		if (youdan.DetectOpenParachute()) {
			break;
		}
	}

	for(int i=0;i<HEAT_LOOP;i++) {
		youdan.HeatNichrome(HEAT_PERIOD*1000, HEAT_CYCLE*1000);
		signal(SIGINT,escape_heat);
	}

}
