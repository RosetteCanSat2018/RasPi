#include <stdint.h>
#include <fstream>

using namespace std;

class Stepping
{
	private:
		int pi;

	public:
		void getPi();
		void decidePin();
		void rotate360(float wait_time);
		/*void hmcInit();
		void mpuInit();
		void mpuSetConfig();
		void hmcSetConfigA(char config);
		void hmcSetConfigB(char config);
		void hmcSetMode(char config);
		void mpuGetMotion6(int16_t output[6]);
		void hmcGetXYZ(int16_t output[3]);*/
};
