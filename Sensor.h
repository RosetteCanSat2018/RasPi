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
		void pigpioStop();
		//HMC5883L_____________________________________________
		void hmcInit();
		void hmcSetConfigA(char config);
		void hmcSetConfigB(char config);
		void hmcSetMode(char config);
		void hmcGetXYZ(double Mg[3]);
		//MPU6050______________________________________________
		void mpuInit();
		void mpuSetConfig();
		void mpuGetMotion6(double ACGY[6]);
		//GPS__________________________________________________
		void GPSGetLine(float data[2]);
		float GPStrunc(float v);
		void GPSread();
};
