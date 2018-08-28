/*
The MIT License
Copyright (c) 2015 Avi Singh
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
*/

#include "vo_features.h"

using namespace cv;
using namespace std;

#define MAX_FRAME 1000
#define MIN_NUM_FEAT 2000

// IMP: Change the file directories (4 places) according to where your dataset is saved before running!
/*
double getAbsoluteScale(int frame_id, int sequence_id, double z_cal) {

	string line;
	int i = 0;
	ifstream myfile("/home/pi/BBM/ImageProcessing/KITTI_VO/00.txt");
	double x = 0, y = 0, z = 0;
	double x_prev, y_prev, z_prev;
	if (myfile.is_open())
	{
		while ((getline(myfile, line)) && (i <= frame_id))
		{
			z_prev = z;
			x_prev = x;
			y_prev = y;
			std::istringstream in(line);
			//cout << line << '\n';
			for (int j = 0; j<12; j++) {
				in >> z;
				if (j == 7) y = z;
				if (j == 3)  x = z;
			}

			i++;
		}
		myfile.close();
	}

	else {
		cout << "Unable to open file";
		return 0;
	}

	return sqrt((x - x_prev)*(x - x_prev) + (y - y_prev)*(y - y_prev) + (z - z_prev)*(z - z_prev));

}
*/

int main(int argc, char** argv) {

	Mat img_1, img_2; //二枚の画像
	Mat R_f, t_f; //the final rotation and tranlation vectors containing the 

	//ofstream myfile;　//書き込み用ファイル
	//myfile.open("results1_1.txt"); //書き込み用ファイル

	//double scale = 1.00;
	char filename1[200];
	char filename2[200];
	sprintf(filename1, "/home/pi/BBM/ImageProcessing/KITTI_VO/00/image_2/%06d.png", 0);//filename1に文字配列を格納
	sprintf(filename2, "/home/pi/BBM/ImageProcessing/KITTI_VO/00/image_2/%06d.png", 1);//filename2に文字配列を格納

	/*
	char text[100];
	int fontFace = FONT_HERSHEY_PLAIN;
	double fontScale = 1;
	int thickness = 1;
	cv::Point textOrg(10, 50);　//テキストをセンタリング
	*/

	//read the first two frames from the dataset
	Mat img_1_c = imread(filename1); //ファイルから画像を読み込み
	Mat img_2_c = imread(filename2); //ファイルから画像を読み込み　

	if (!img_1_c.data || !img_2_c.data) {
		std::cout << " --(!) Error reading images " << std::endl; return -1;
	}

	// we work with grayscale images
	cvtColor(img_1_c, img_1, COLOR_BGR2GRAY); //２値下
	cvtColor(img_2_c, img_2, COLOR_BGR2GRAY); //２値下

	// feature detection, tracking
	vector<Point2f> points1, points2;        //vectors to store the coordinates of the feature points
	featureDetection(img_1, points1);        //detect features in img_1
	vector<uchar> status;
	featureTracking(img_1, img_2, points1, points2, status); //track those features to img_2

															 //TODO: add a fucntion to load these values directly from KITTI's calib files
															 // WARNING: different sequences in the KITTI VO dataset have different intrinsic/extrinsic parameters
	double focal = 718.8560; //焦点距離$$$$$$$$$$$$$$$$$キャリブレーションの必要あり$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
	//cv::Point2d pp(607.1928, 185.2157); //２次元上の点を表す
	//recovering the pose and the essential matrix
	Mat E, R, t, mask;
	E = findEssentialMat(points2, points1, focal, pp, RANSAC, 0.999, 1.0, mask); //RANSAC:ノイズ除去,Essential行列の算出
	recoverPose(E, points2, points1, R, t, focal, pp, mask); //カメラの並進と回転を求める

	Mat prevImage = img_2; //画像２を格納
	Mat currImage; //新たな画像を取り込むための変数
	vector<Point2f> prevFeatures = points2; //vectors to store the coordinates of the feature points
	vector<Point2f> currFeatures; 

	char filename[100];

	R_f = R.clone(); //Matの深いコピー
	t_f = t.clone(); //Matの深いコピー

	clock_t begin = clock(); //CPU時間の記録

	//namedWindow("Road facing camera", WINDOW_AUTOSIZE);// Create a window for display.
	//namedWindow("Trajectory", WINDOW_AUTOSIZE);// Create a window for display.

	Mat traj = Mat::zeros(600, 600, CV_8UC3); //600×600のカラー画像の生成

	for (int numFrame = 2; numFrame < MAX_FRAME; numFrame++) {　//画像を読み込んで処理しているためwhile文に書き直す
		sprintf(filename, "/home/pi/BBM/ImageProcessing/KITTI_VO/00/image_2/%06d.png", numFrame); //filenameに文字配列を格納
		//cout << numFrame << endl;
		Mat currImage_c = imread(filename); //ファイルから画像読み込み
		cvtColor(currImage_c, currImage, COLOR_BGR2GRAY); //二値化
		vector<uchar> status;　 //vectors to store the coordinates of the feature points
		featureTracking(prevImage, currImage, prevFeatures, currFeatures, status);　//特徴点検出

		E = findEssentialMat(currFeatures, prevFeatures, focal, pp, RANSAC, 0.999, 1.0, mask); //RANSAC:ノイズ除去,Essential行列の算出
		recoverPose(E, currFeatures, prevFeatures, R, t, focal, pp, mask); //カメラの並進と回転を求める

		Mat prevPts(2, prevFeatures.size(), CV_64F), currPts(2, currFeatures.size(), CV_64F); 


		for (int i = 0; i<prevFeatures.size(); i++) {   //this (x,y) combination makes sense as observed from the source code of triangulatePoints on GitHub
			prevPts.at<double>(0, i) = prevFeatures.at(i).x; //座標の画素値にアクセス
			prevPts.at<double>(1, i) = prevFeatures.at(i).y;

			currPts.at<double>(0, i) = currFeatures.at(i).x;
			currPts.at<double>(1, i) = currFeatures.at(i).y;
		}

		// scale = getAbsoluteScale(numFrame, 0, t.at<double>(2));

		//cout << "Scale is " << scale << endl;

		/*
		if ((scale>0.1) && (t.at<double>(2) > t.at<double>(0)) && (t.at<double>(2) > t.at<double>(1))) {

			t_f = t_f + scale * (R_f*t);
			R_f = R * R_f;

		}

		else {
			//cout << "scale below 0.1, or incorrect translation" << endl;
		}
		*/

		// lines for printing results
		// myfile << t_f.at<double>(0) << " " << t_f.at<double>(1) << " " << t_f.at<double>(2) << endl;

		// a redetection is triggered in case the number of feautres being trakced go below a particular threshold
		if (prevFeatures.size() < MIN_NUM_FEAT) {
			//cout << "Number of tracked features reduced to " << prevFeatures.size() << endl;
			//cout << "trigerring redection" << endl;
			featureDetection(prevImage, prevFeatures);
			featureTracking(prevImage, currImage, prevFeatures, currFeatures, status);

		}

		prevImage = currImage.clone();
		prevFeatures = currFeatures;

		/*
		int x = int(t_f.at<double>(0)) + 300;
		int y = int(t_f.at<double>(2)) + 100;
		circle(traj, Point(x, y), 1, CV_RGB(255, 0, 0), 2);

		rectangle(traj, Point(10, 30), Point(550, 50), CV_RGB(0, 0, 0), CV_FILLED);
		sprintf(text, "Coordinates: x = %02fm y = %02fm z = %02fm", t_f.at<double>(0), t_f.at<double>(1), t_f.at<double>(2));
		putText(traj, text, textOrg, fontFace, fontScale, Scalar::all(255), thickness, 8);

		imshow("Road facing camera", currImage_c);
		imshow("Trajectory", traj);
		*/

		waitKey(1);

	}

	clock_t end = clock();
	double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
	cout << "Total time taken: " << elapsed_secs << "s" << endl;

	//cout << R_f << endl;
	//cout << t_f << endl;

	return 0;
}