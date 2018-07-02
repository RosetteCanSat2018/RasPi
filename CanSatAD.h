#ifndef _CANSATAD_H_
#define _CANSATAD_H_

#include "EKF.h"

#include "Eigen/Core"
#include "Eigen/LU"

#define _USE_MATH_DEFINES
#include <math.h>

using namespace Eigen;

class CanSatAD : public EKF
{
public:

	// スーパークラスのオーバーライド
	void SetParameter(double magnet_origin_x, double magnet_origin_y, double magnet_origin_z);

	// 初期値収束までのセンサ値格納
	void GetValue(int loop_number, double ACGY[], double Mg[]);

	// オフセット処理(ジャイロ)
	void OffsetGyro();

	// オフセット処理(地磁気)
	void OffsetMagnet();

	// 事前解析
	void PreAnalysis();

	// 姿勢推定時のセンサ値格納
	void GetValue(double ACGY[], double Mg[]);

	// 内部変数の初期値(gamma: 共分散行列の初期値)
	void SetInitial(int gamma);

	// 行列A
	void MatrixA();

	// 行列B
	void MatrixB();

	// 行列C
	void MatrixC();

	// 関数h
	void FunctionH();

	// 方向余弦行列(関数h内で使うもの)
	MatrixXd ToDCMtemp(VectorXd q);

	// 方向余弦行列(最終的に出力するもの)
	void ToDCM(VectorXd q);

	// 正規化
	VectorXd Normalize(VectorXd x);

	// カルマンゲインの修正
	void KalmanGainCorrect(VectorXd y_value);

	// カルマンフィルタ(予測＋フィルタリング)
	void KalmanFilter(VectorXd y_value);

	// クォータニオン，共分散行列の適正値算出
	void Converge(int loop_number);

	// 姿勢推定
	void AtittudeEstimate();

	// 内部変数を返す
	VectorXd GetStateVariable();

	// DCNを返す
	MatrixXd GetDCM();

	void Get();

	void Close();

private:
	const double frequency = 0.01;
	const int stay = 41728; // 収束するまでに必要な静止状態のデータ数
	MatrixXd accel_array;
	MatrixXd gyro_array;
	MatrixXd magnet_array;

	VectorXd gyro_average;

	VectorXd magnet_origin;

	double magnet_norm;
	double accel_norm;
	double inclination;

	MatrixXd y_array;

	int gamma; // 共分散行列の初期値

	MatrixXd DCM;

	FILE *fp2;


};

#endif