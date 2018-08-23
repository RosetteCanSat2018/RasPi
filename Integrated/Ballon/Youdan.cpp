#include "pigpio.h"
#include "Youdan.h"
#include "unistd.h"
#include <iostream>
#include <signal.h>

using namespace std;

void Youdan::Init(int _read_signal, int _write_signal, int _signal_to_FET)
{
	read_signal = _read_signal;
	write_signal = _write_signal;
	signal_to_FET = _signal_to_FET;

	gpioInitialise();
	gpioSetMode(read_signal, PI_INPUT);
	gpioSetMode(write_signal, PI_OUTPUT);
	gpioSetMode(signal_to_FET, PI_OUTPUT);

	gpioWrite(signal_to_FET, 0);
	gpioWrite(write_signal, 1);
}

int Youdan::DetectOpenParachute()
{
	cout << gpioRead(read_signal) << endl;
	if (gpioRead(read_signal)){return 1;}
	else{return 0;}
}

void Youdan::escape_program(int sig){
	cout << "ctrl+c" << endl;
	gpioWrite(signal_to_FET, 0);
	exit(1);
}

void Youdan::HeatNichrome(unsigned int waittime,unsigned int looptime)
{

	gpioWrite(signal_to_FET, 1);

	usleep(waittime);

	gpioWrite(signal_to_FET, 0);

	usleep(looptime);

}
