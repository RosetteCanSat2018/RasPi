#include "MU2.h"
#include <pigpio.h>
#include "MU2.h"
#include <pigpio.h>
#include <string.h>
#include <iostream>
#include <unistd.h> //for sleep()

using namespace std;

int main() {
	gpioInitialise();
	MU2 mu;
	
	char data[] = "STARWARS";
	
	mu.setPin();
	//mu.MU2Initialise();	
	
	for(int i=0;i<7;i++){
		mu.Send(data);
		usleep(100000);
		}
		
	mu.Send("END Program");
        
    //serClose(handle);	

}

