#include "pigpio.h"
#include "RPi_Sensor.h"
#include "RPi_BNO055.h"
#include "imumaths.h"
#include <iostream>

Adafruit_BNO055 bno = Adafruit_BNO055();

using namespace std;
//using namespace Eigen;



double angle;

int main(void)
{
	bno.Init(100); //sample rate[ms]
	//bno.SerialInit();

	while(1)
	{
		bno.AttitudeDeterminate();
		//bno.PrintQ();
		//bno.PrintDCM();
		cout << bno.GetAngle() << endl;
		//bno.SerialPrint();
	}

}
