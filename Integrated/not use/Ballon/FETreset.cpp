#include "pigpio.h"


#include "unistd.h"

#include <iostream>



using namespace std;

int main () {
	
gpioInitialise();


	gpioSetMode(4, PI_OUTPUT);
	//gpioSetMode(22, PI_OUTPUT);
	
	gpioWrite(4, 0);
	
	//gpioWrite(22, 0);
	
	}
