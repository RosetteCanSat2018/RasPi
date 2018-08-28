#include "MU2.h"
#include <pigpio.h>
#include <string.h>
#include <iostream>
#include <unistd.h> //for sleep()
#include "Sensor.h"
#include <signal.h>

using namespace std;

MU2 mu;
Sensor sensor;

void escape_program(int sig);

int main() {
	char c_data[8];
	gpioInitialise();


	//char data[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZ@0123456789";
	//char data[] = "ABCDEFGHIJKLMN";
	//float data[2] = {0.1,0.5};

	//mu.setPin();
	//mu.MU2Initialise();

	mu.Send("START Program");
	sleep(1);

	float GPSf[2];
	while(1){
		signal(SIGINT,escape_program);
		sensor.GPSGetData_f(GPSf);
		cout << GPSf[1] << endl;
		//mu.Send("12345");
		sleep(1);
		mu.SendGPS(GPSf[0]);
		mu.SendGPS(GPSf[1]);
		sleep(3);
		cout << "1" << endl;


	}


	/*
	for(int i=0;i<7;i++){
		mu.Send(c_data);
		}
	*/

	mu.Send("END Program");


    //serClose(handle);


}
void escape_program(int sig){
	cout << "escape program" <<endl;
	mu.Send("END Program");
	mu.closeMU2();
  exit(1);
  }
