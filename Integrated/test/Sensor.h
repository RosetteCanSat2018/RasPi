#include <stdint.h>
#include <fstream>
#include <string>

using namespace std;

class Sensor
{
	private :
		int MPL3115A2_i2c;
		int rc;
		int GPS_handle;
	
	public:
		float longitude,latitude;
		void  pigpioStop();

		//MPL3115A2---------------------------------------------
		void mplSetConfig();
		double mplGetALT(double sea_pressure);

		//GPS---------------------------------------------------
		void GPSGetData(char c_data[2]);
		void GPSGetData_f(float data[2]);
};
