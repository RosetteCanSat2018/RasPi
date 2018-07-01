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
		void ClockwiseRotation(int cycle);
		void CounterClockwiseRotation(int cycle);
		void ClockwiseTick(int count);
		void CounterClockwiseTick(int count);
		void control(int rotation);

		/*void hmcInit();
		void mpuInit();
		void mpuSetConfig();
		void hmcSetConfigA(char config);
		void hmcSetConfigB(char config);
		void hmcSetMode(char config);
		void mpuGetMotion6(int16_t output[6]);
		void hmcGetXYZ(int16_t output[3]);*/
};
