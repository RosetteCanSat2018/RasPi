#include <pigpio.h> 
#include "MU2.h"
#include <string.h>
#include <unistd.h>
#include <iostream>

using namespace std;

int main() {

	MU2 mu;
    gpioInitialise();
	char data[10] = "@DT04JAXA";
	mu.setPin();
	mu.MU2Initialise();
    for(int i=0;i<20;i++){
		mu.Send(data);
        cout << i << endl;        
        sleep(1);
        }
    
}
