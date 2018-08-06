
#include "MU2.h"
#include <pigpio.h>
#include <string.h>
#include <iostream>
#include <unistd.h> //for sleep()


using namespace std;

int main() {
	gpioInitialise();
	
///////////////////////////////////
	
	
	MU2 mu;
	unsigned char data[10] = "@DT04JAXA";
	//mu.setPin();
	//mu.MU2Initialise();
	int handle = serOpen("/dev/ttyS0", 19200, 0);
    for(int i=0;i<7;i++){
		//mu.Send(data);
		int a,b,c;
		/*
		a = serWriteByte(handle, 97);
		cout << "a=" << a << endl;
		usleep(10000);
		b = serWriteByte(handle, 0x61);
		cout << "b=" << b << endl;
		usleep(20000);
		c = serWriteByte(handle, 'a');
		cout << "c=" << c << endl;
		usleep(30000);
		cout << i << endl; 
        usleep(50000);
        */
        serWriteByte(handle, '@');
		usleep(10000);
		serWriteByte(handle, 'D');
		usleep(10000);
		serWriteByte(handle, 'T');
		usleep(10000);
		serWriteByte(handle, '0');
		usleep(10000);
		serWriteByte(handle, '1');
		usleep(10000);
		serWriteByte(handle, 'J');
		usleep(10000);
		serWriteByte(handle, '\r');
		usleep(10000);
		serWriteByte(handle, '\n');
		usleep(10000);
		cout << i << endl; 
        usleep(1000000);
        }
    serClose(handle);	
 ///////////////////////////////   
 /*
    int handle;
    //gpioSetMode(14,PI_OUTPUT);
    handle = serOpen("/dev/ttyS0", 115200, 0);
    cout << "handle=" << handle << endl;
    
    for(int i=0;i<5;i++){
	int a,b,c;
	a = serWriteByte(handle, 97);
	cout << "a=" << a << endl;
	b = serWriteByte(handle, 0x61);
	cout << "b=" << b << endl;
	c = serWriteByte(handle, '61');
	cout << "c=" << c << endl;
	
	cout << "count=" << i << endl;
	sleep(1);
	}
*/
	
}
