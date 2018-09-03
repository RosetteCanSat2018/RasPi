class Youdan
{
private:
	int read_signal;
	int write_signal;
	int signal_to_FET;
	//static int signal_to_FET;

public:
	void Init(int read_signal, int write_signal, int signal_to_FET);
	int DetectOpenParachute();
	void HeatNichrome(unsigned int waittime, unsigned int looptime);
	void escape_program(int sig);
	//void escape_program(int sig);
};
