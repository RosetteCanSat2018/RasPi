#include <stdint.h>
#include <fstream>

using namespace std;

class Sensor
{
	private :
		int pi;
		int MPU6050_i2c;
		int HMC5883L_i2c;
		double Mg[3],ACGY[6];
		char msg[256];
	
	public:
		float longitude,latitude;
	
		void getPi();
		void hmcInit();
		void mpuInit();
		void mpuSetConfig();
		void hmcSetConfigA(char config);
		void hmcSetConfigB(char config);
		void GPSConfig();
		void hmcSetMode(char config);
		void GetMotion9(double Sdata[9]);
		void pigpioStop();
		void GPSGetData(float data[2]);
		float GPStrunc(float v);
};
