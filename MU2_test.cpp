#include "MU2.h"
#include <pigpio.h>
#include <string.h>
#include <iostream>
#include <unistd.h> //for sleep()

using namespace std;

int main() {
	gpioInitialise();
	MU2 mu;
	
	//char data[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZ@0123456789";
	char data[] = "ABCDEFGHIJKLMN";
	//float data[2] = {0.1,0.5}; 
	
	mu.setPin();
	//mu.MU2Initialise();
	
	
	for(int i=0;i<7;i++){
		mu.Send(data);
		}
		
	mu.Send("END Program");
        
       
    //serClose(handle);	

	
}
