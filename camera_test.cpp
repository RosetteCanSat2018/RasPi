#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/videoio/videoio.hpp>
#include <iostream>
#include <pigpio.h>
using namespace cv;
using namespace std;

int main(int argc, char* argv[])
{
	gpioInitialise();
	int s1, m1, s2, m2;

	Mat mat;
	VideoCapture vcap(0);
	int width, height, fourcc;
	double fps;
	width = (int)vcap.get(cv::CAP_PROP_FRAME_WIDTH);   // フレーム横幅を取得
	height = (int)vcap.get(cv::CAP_PROP_FRAME_HEIGHT);  // フレーム縦幅を取得
	fps = vcap.get(cv::CAP_PROP_FPS);                        // フレームレートを取得
	fourcc = cv::VideoWriter::fourcc('H', '2', '6', '4');       // H.264 / .wmv
	cv::VideoWriter writer;
	writer.open("test.avi", fourcc, fps, cv::Size(width, height));
	cv::Mat frame, dst;


	if (!vcap.isOpened())
	{
		cout << "vcap is not open." << endl;
		return -1;
	}

	gpioTime(0, &s1, &m1);

	while (1)
	{
		vcap >> frame;
		if (frame.empty() == true)
		{
			cout << "frame empty" << endl;
			break;
		}
		cv::flip(frame, dst, 1);
		writer << dst;
		cv::waitKey(1000/60);

		gpioTime(0, &s2, &m2);
		if (s2-s1 >= 10)
		{
			cout << "time up" << endl;
			break;
		}
	}
	return 0;
}

//header file  -I/usr/local/include/opencv
//library      -L/usr/local/lib
//g++ camera_test.cpp -o test -I/usr/local/include -L/usr/local/lib -lopencv_core -lopencv_highgui -lopencv_videoio