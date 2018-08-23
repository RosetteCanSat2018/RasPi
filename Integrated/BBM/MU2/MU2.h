#ifndef MU2_H
#define MU2_H 

class MU2
{
private:
	int MU2_handle;
    int a;
    int b;
    int c;

public:
	int data_len;
	int data_num;
	void setPin();
	void MU2Initialise();
    void SendTerminal();
	void Send(char data[]);
};


#endif 


