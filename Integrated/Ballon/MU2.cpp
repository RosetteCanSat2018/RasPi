#include "MU2.h"
#include <string.h>
#include <pigpio.h>
#include <iostream>
#include <unistd.h>
#include <string>
using namespace std;

#include <string>

void MU2::setPin()
{
	//MU2_handle = serOpen("/dev/ttyS0", 19200, 0);
	//cout << "MU2_handle=" << MU2_handle << endl;
}

void MU2::SendTerminal()
{
	a = serWriteByte(MU2_handle, '\r');
	//cout << "a=" << a << endl;
	b = serWriteByte(MU2_handle, '\n');
	//cout << "b=" << b << endl;
}

void MU2::MU2Initialise()
{
	serWrite(MU2_handle, "@CH08", 5);
	SendTerminal();
	serWrite(MU2_handle, "@GI01", 5);
	SendTerminal();
	serWrite(MU2_handle, "@EI03", 5);
	SendTerminal();
	serWrite(MU2_handle, "@DI01", 5);
	SendTerminal();
}

void MU2::Send(char data[])
{
	MU2_handle = serOpen("/dev/ttyS0", 19200, 0);
	char i_data[6];
	i_data[0] = '@';
	i_data[1] = 'D';
	i_data[2] = 'T';

	data_len = strlen(data);
	data_num = data_len;
	cout << "data_len =" << data_len << endl;
	for (int i = 4; i>2; i--) {
		if (data_len % 16<10) {
			i_data[i] = '0' + data_len % 16;
		}
		else {
			switch (data_len % 16) {
			case 10:i_data[i] = 'A'; break;
			case 11:i_data[i] = 'B'; break;
			case 12:i_data[i] = 'C'; break;
			case 13:i_data[i] = 'D'; break;
			case 14:i_data[i] = 'E'; break;
			case 15:i_data[i] = 'F'; break;
			}
		}
		data_len /= 16;
	}
	for (int counter = 0; counter < 5; counter++) {
		serWriteByte(MU2_handle, i_data[counter]);
		cout << "i_data [" << counter << "] = " << i_data[counter] << endl;
		usleep(1000);
	}

	for (int data_counter = 0; data_counter < data_num; data_counter++) {
		serWriteByte(MU2_handle, data[data_counter]);
		cout << "data [" << data_counter << "] = " << data[data_counter] << endl;
		usleep(1000);
	}
	SendTerminal();


	//for(int i=0; i<8; i++){
	//int c = serReadByte(MU2_handle);
	//cout << "c= " << c << endl;
	//}
	serClose(MU2_handle);

}
/*
void MU2::SendGPS(float data) {
	MU2_handle = serOpen("/dev/ttyS0", 19200, 0);
	string s_GPS;
	s_GPS = to_string(data);
	char c_GPS[8] = {};
	s_GPS.copy(c_GPS, 8);
	for (int i=0; i<8, i++) {
		serWriteByte(MU2_handle, c_GPS[i]);
		usleep(1000);
	}
	//serWrite(MU2_handle, c_GPS, 8);
	SendTerminal();
	serClose(MU2_handle);
}
*/
void MU2::SendGPS(float data) {
	//MU2_handle = serOpen("/dev/ttyS0", 19200, 0);
	string s_GPS;
	int length;
	s_GPS = to_string(data);
	length = s_GPS.length();
	char c_GPS[length+1] = {};
	s_GPS.copy(c_GPS, length);
	Send(c_GPS);
	/*
	for (int i = 0; i<7; i++) {
		serWriteByte(MU2_handle, c_GPS[i]);
		cout << c_GPS[i] << endl;
		usleep(1000);
	}
	*/
	//serWrite(MU2_handle, c_GPS, 8);
	//SendTerminal();
	//serClose(MU2_handle);
}

void MU2::closeMU2() {
	int close = serClose(MU2_handle);
	cout << close << endl;
}
