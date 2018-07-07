#include <MU2.h>
#include <string.h>


//char data[10] = "@DT04JAXA";
char terminal1 = '\r';
char terminal2 = '\n';

void MU2::getPi()
{
	pi = pigpio_start(0, 0);
}


void MU2::setPin()
{
	int gpioSetMode(MU2_pin, PI_OUTPUT);
	s_open = seriai_open(pi,'/dev/ttyAMA0', 19200, 0)
}

void MU2::Send(char data[])
{
	char send_data[] = strcat(data, terminal1);///add terminal"\r" to data
	send_data[] = strcat(send_data, terminal2);///add terminal"\n" to data

	//send_data_len = strlen(send_data);

	for (send_counter = 0; send_counter < send_data_len - 1; send_counter++) {
		int serial_write_byte(pi, s_open, 0xsend_data[counter]);
	}
}