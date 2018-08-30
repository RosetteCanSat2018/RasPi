#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/videoio/videoio.hpp>
#include <iostream>
#include <pigpio.h>
#include <stdio.h>
#include "vo_features.h"

using namespace cv;
using namespace std;

#define MAX_FRAME 1000
#define MIN_NUM_FEAT 2000

int main() {

	gpioInitialise();
	int s1, m1, s2, m2;
	int width, height, fourcc;
	double fps, i = 0;
	FILE *fp;

	cv::Mat frame, frame1, frame2, dst;
	cv::Mat img_1_c, img_2_c;
	cv::Mat img_1, img_2;
	cv::Mat R_f, t_f;

	cv::VideoCapture vcap;
	cv::VideoWriter writer;

	vcap.open(0);
	fp = fopen("camera_fps_calc.csv", "w");

	width = (int)vcap.get(cv::CAP_PROP_FRAME_WIDTH);
	height = (int)vcap.get(cv::CAP_PROP_FRAME_HEIGHT);
	fps = vcap.get(cv::CAP_PROP_FPS);
	//fourcc = writer.fourcc('X', 'V', 'I', 'D');

	//writer.open("test.avi", fourcc, 13, cv::Size(width, height));

	if (!vcap.isOpened()) {
		cout << "vcap is not open." << endl;
		return -1;
	}

	gpioTime(0, &s1, &m1);

	//------------------------------------------------
	vcap >> frame1;
	cv::flip(frame1, img_1_c, 1);
	waitKey(1);
	vcap >> frame2;
	cv::flip(frame2, img_2_c, 1);

	// we work with grayscale images
	cvtColor(img_1_c, img_1, COLOR_BGR2GRAY);
	cvtColor(img_2_c, img_2, COLOR_BGR2GRAY);

	// feature detection, tracking
	vector<Point2f> points1, points2;        //vectors to store the coordinates of the feature points
	featureDetection(img_1, points1);        //detect features in img_1
	vector<uchar> status;
	featureTracking(img_1, img_2, points1, points2, status); //track those features to img_2

	double focal = 3.04;
	cv::Point2d pp(0.0, 0.0);
	//recovering the pose and the essential matrix
	Mat E, R, t, mask;
	E = findEssentialMat(points2, points1, focal, pp, RANSAC, 0.999, 1.0, mask);
	recoverPose(E, points2, points1, R, t, focal, pp, mask);

	Mat prevImage = img_2;
	Mat currImage;
	vector<Point2f> prevFeatures = points2; //vectors to store the coordinates of the feature points
	vector<Point2f> currFeatures;

	//char filename[100];

	R_f = R.clone();
	t_f = t.clone();

	//clock_t begin = clock(); 

	//---------------------------------------------

	while (1) {
		cv::Mat currImage_c;
		vcap >> frame;
		/*
		if (frame.empty() == true){
		cout << "frame empty" << endl;
		break;
		}
		*/
		cv::flip(frame, currImage_c, 1);
		cvtColor(currImage_c, currImage, COLOR_BGR2GRAY);
		vector<uchar>status;
		featureTracking(prevImage, currImage, prevFeatures, currFeatures, status);

		E = findEssentialMat(currFeatures, prevFeatures, focal, pp, RANSAC, 0.999, 1.0, mask);
		recoverPose(E, currFeatures, prevFeatures, R, t, focal, pp, mask);

		//Mat prevPts(2, prevFeatures.size(), CV_64F), currPts(2, currFeatures.size(), CV_64F);

		R_f = R * R_f;

		if (prevFeatures.size() < MIN_NUM_FEAT) {
			//cout << "Number of tracked features reduced to " << prevFeatures.size() << endl;
			//cout << "trigerring redection" << endl;
			featureDetection(prevImage, prevFeatures);
			featureTracking(prevImage, currImage, prevFeatures, currFeatures, status);
		}

		prevImage = currImage.clone();
		prevFeatures = currFeatures;

		i++;

		cout << R_f << endl;
		printf("%d.%03d seconds\r\n", s2-s1, (m2-m1) / 1000);

		gpioTime(0, &s2, &m2);
		if (s2 - s1 >= 720) {
			fprintf(fp, "start:%d.%03dseconds\r\n", s1, m1 / 1000);
			fprintf(fp, "stop:%d.%03dseconds\r\n", s2, m2 / 1000);
			fprintf(fp, "loopnumber is %f\r\n", i);
			fclose(fp);
			break;
		}
	}
	cout << "end program" << endl;
	return 0;
}
