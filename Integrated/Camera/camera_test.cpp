#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/videoio/videoio.hpp>
#include <iostream>
#include <pigpio.h>
#include <stdio.h>

using namespace cv;
using namespace std;

int main(int argc, char* argv[])
{
	gpioInitialise();
	int s1, m1, s2, m2;
	int width, height, fourcc;
	double fps,i;
	FILE *fp;
	
	cv::Mat frame, dst;
	cv::VideoCapture vcap;
	cv::VideoWriter writer;

	vcap.open(0);
	fp = fopen("camera_fps_calc.csv","w");

	width = (int)vcap.get(cv::CAP_PROP_FRAME_WIDTH);   // フレーム横幅を取得
	height = (int)vcap.get(cv::CAP_PROP_FRAME_HEIGHT);  // フレーム縦幅を取得
	fps = vcap.get(cv::CAP_PROP_FPS);                        // フレームレートを取得
	fourcc = writer.fourcc('X', 'V', 'I', 'D');      

	writer.open("test.avi", fourcc, 13, cv::Size(width, height));
	
	if (!vcap.isOpened())
	{
		cout << "vcap is not open." << endl;
		return -1;
	}

	gpioTime(0, &s1, &m1);

	while (1)
	{
		vcap >> frame;
		/*
		if (frame.empty() == true)
		{
			cout << "frame empty" << endl;
			break;
		}
		*/
		cv::flip(frame, dst, 1);
		writer << dst;
		i++;
		//cv::waitKey(10);
		gpioTime(0, &s2, &m2);
		if (s2-s1 >= 720)
		{
			writer.release();
			fprintf(fp,"start:%d.%03dseconds\r\n",s1,m1/1000);
			fprintf(fp,"stop:%d.%03dseconds\r\n",s2,m2/1000);
			fprintf(fp,"loopnumber is %f\r\n",i);
			break;
		}
	}
	cout << "end program" << endl;
	return 0;
}
//g++ camera_test.cpp -o test -I/usr/local/include -L/usr/local/lib -lopencv_core -lopencv_highgui -lopencv_videoio -lpigpio -lrt


//header file  -I/usr/local/include/opencv
//library      -L/usr/local/lib
//g++ camera_test.cpp -o test -I/usr/local/include -L/usr/local/lib -lopencv_core -lopencv_highgui -lopencv_videoio -lpigpio -lrt


