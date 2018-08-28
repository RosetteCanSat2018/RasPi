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

int main(){

	gpioInitialise();
	int s1, m1, s2, m2;
	int width, height, fourcc;
	double fps, i=0;
	FILE *fp;

	cv::Mat frame, frame1, frame2, dst;
	cv::Mat img_1_c, img_2_c;//�񖇂̉摜
	cv::Mat img_1, img_2; //�񖇂̉摜(��l����)
	cv::Mat R_f, t_f; //the final rotation and tranlation vectors containing the

	cv::VideoCapture vcap;
	cv::VideoWriter writer;

	vcap.open(0);
	fp = fopen("camera_fps_calc.csv", "w");

	width = (int)vcap.get(cv::CAP_PROP_FRAME_WIDTH);   // �t���[���������擾
	height = (int)vcap.get(cv::CAP_PROP_FRAME_HEIGHT);  // �t���[���c�����擾
	fps = vcap.get(cv::CAP_PROP_FPS);                        // �t���[�����[�g���擾
	fourcc = writer.fourcc('X', 'V', 'I', 'D');

	writer.open("test.avi", fourcc, 13, cv::Size(width, height));

	if (!vcap.isOpened()){
		cout << "vcap is not open." << endl;
		return -1;
	}

	gpioTime(0, &s1, &m1);

	/////////////////////////�P���ځC�Q���ڂ�����////////////////////////////
	vcap >> frame1;
	cv::flip(frame1, img_1, 1);
	waitKey(1);
	vcap >> frame2;
	cv::flip(frame2, img_2, 1);

	// we work with grayscale images
	cvtColor(img_1_c, img_1, COLOR_BGR2GRAY); //��l��
	cvtColor(img_2_c, img_2, COLOR_BGR2GRAY); //��l��

	// feature detection, tracking
	vector<Point2f> points1, points2;        //vectors to store the coordinates of the feature points
	featureDetection(img_1, points1);        //detect features in img_1
	vector<uchar> status;
	featureTracking(img_1, img_2, points1, points2, status); //track those features to img_2

	double focal = 718.8560; //$$$$$$$$$$$$$$$$$$$�J�����̃L�����u���[�V�����̕K�v����$$$$$$$$$$$$$$$$$$$$$$$$
							 //cv::Point2d pp(607.1928, 185.2157); //�Q������̓_��\��
							 //recovering the pose and the essential matrix
	Mat E, R, t, mask;
	E = findEssentialMat(points2, points1, focal, pp, RANSAC, 0.999, 1.0, mask); //RANSAC:�m�C�Y����,Essential�s��̎Z�o
	recoverPose(E, points2, points1, R, t, focal, pp, mask); //�J�����̕��i�Ɖ�]�����߂�

	Mat prevImage = img_2; //�摜�Q���i�[
	Mat currImage; //�V���ȉ摜����荞�ނ��߂̕ϐ�
	vector<Point2f> prevFeatures = points2; //vectors to store the coordinates of the feature points
	vector<Point2f> currFeatures;

	char filename[100];

	R_f = R.clone(); //Mat�̐[���R�s�[
	t_f = t.clone(); //Mat�̐[���R�s�[

	clock_t begin = clock(); //CPU���Ԃ̋L�^

	/////////////////////////3���ڈȍ~������////////////////////////////

	while (1){
		cv::Mat currImage_c; //�t�@�C������摜�ǂݍ���
		vcap >> frame;
		/*
		if (frame.empty() == true){
		cout << "frame empty" << endl;
		break;
		}
		*/
		cv::flip(frame, currImage_c, 1);
		cvtColor(currImage_c, currImage, COLOR_BGR2GRAY); //��l��
		vector<uchar> status;�@ //vectors to store the coordinates of the feature points
		featureTracking(prevImage, currImage, prevFeatures, currFeatures, status);�@//�����_���o
		
		E = findEssentialMat(currFeatures, prevFeatures, focal, pp, RANSAC, 0.999, 1.0, mask); //RANSAC:�m�C�Y����,Essential�s��̎Z�o
		recoverPose(E, currFeatures, prevFeatures, R, t, focal, pp, mask); //�J�����̕��i�Ɖ�]�����߂�

		Mat prevPts(2, prevFeatures.size(), CV_64F), currPts(2, currFeatures.size(), CV_64F);

		if (prevFeatures.size() < MIN_NUM_FEAT) {
			//cout << "Number of tracked features reduced to " << prevFeatures.size() << endl;
			//cout << "trigerring redection" << endl;
			featureDetection(prevImage, prevFeatures);
			featureTracking(prevImage, currImage, prevFeatures, currFeatures, status);
		}

		prevImage = currImage.clone(); //�ŐV�̉摜��prevImage�ֈړ�
		prevFeatures = currFeatures;�@//�ŐV�̓����_��prevFeatures�ֈړ�

		i++; //���[�v�񐔂��J�E���g

		gpioTime(0, &s2, &m2);
		if (s2 - s1 >= 720){
			fprintf(fp, "start:%d.%03dseconds\r\n", s1, m1 / 1000);
			fprintf(fp, "stop:%d.%03dseconds\r\n", s2, m2 / 1000);
			fprintf(fp, "loopnumber is %f\r\n", i);
			break;
		}
	}
	cout << "end program" << endl;
	return 0;
}