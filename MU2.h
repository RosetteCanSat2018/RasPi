#ifndef MU2_H
#define MU2_H 



class MU2
{
private:
	int pi;
	char terminal1;
	char terminal2;
    int MU2_handle;

public:
	int send_data_len;
	int send_counter;
	void setPin();
	void MU2Initialise();
    void SendTerminal();
	void Send(char data[]);
};


#endif 
