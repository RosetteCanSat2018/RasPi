#ifndef MU2_H
#define MU2_H 

class MU2
{
private:
	int pi;
	unsigned char terminal1;
	unsigned char terminal2;
	int MU2_handle;
    int a;
    int b;
    int c;

public:
	int send_data_len;
	int send_counter;
	void setPin();
	void MU2Initialise();
    void SendTerminal();
	void Send(unsigned char data[]);
};


#endif 
