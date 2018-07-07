#ifndef MU2_H
#define MU2_H

#include <pigpiod_if2.h> 

#define MU2_pin 14

class MU2
{
private:
	char terminal1;
	char terminal2;

public:
	int send_data_len;
	int send_counter;
	void getPi();
	void setPin();
	void Send();
};


#endif 
