#include "MU2.h"
#include <string.h>
#include <pigpio.h>
#include <iostream>

#define MU2_pin 14

using namespace std;

char  terminal1 = '\r';
char  terminal2 = '\n';

void MU2::setPin()
{
	gpioSetMode(MU2_pin, 1);
	MU2_handle = serOpen("/dev/ttyS0", 19200, 0);
}

void MU2::SendTerminal()
{
	serWriteByte(MU2_handle, terminal1);
	//cout << terminal1 << endl;
	serWriteByte(MU2_handle, terminal2);
	//cout << terminal2 << endl;
}

void MU2::MU2Initialise()
{
	serWrite(MU2_handle,"@CH08",5);
	SendTerminal();
	serWrite(MU2_handle,"@GI01",5);
	SendTerminal();
	serWrite(MU2_handle,"@EI03",5);
	SendTerminal();
	serWrite(MU2_handle,"@DI01",5);
	SendTerminal();
}

void MU2::Send(char data[])
{
	//send_data_len = strlen(send_data);
	
	
    send_data_len = 9;

	for (send_counter = 0; send_counter < send_data_len ; send_counter++) {
		serWriteByte(MU2_handle,data[send_counter]);
        cout << data[send_counter] << endl;
	}
	
	//serWrite(MU2_handle,data,10);
	SendTerminal();
	
	/*
        serWriteByte(MU2_handle,'@');
        serWriteByte(MU2_handle,'D');
        serWriteByte(MU2_handle,'T');
        serWriteByte(MU2_handle,'0');
        serWriteByte(MU2_handle,'6');
        serWriteByte(MU2_handle,'a');
        serWriteByte(MU2_handle,'b');
        serWriteByte(MU2_handle,'c');
        serWriteByte(MU2_handle,'1');
        serWriteByte(MU2_handle,'2');
        serWriteByte(MU2_handle,'3');
        serWriteByte(MU2_handle,'\r');
        serWriteByte(MU2_handle,'\n');
     */
}

