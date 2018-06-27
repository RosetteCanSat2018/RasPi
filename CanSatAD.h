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
	void SetParameter(int magnet_origin_x, int magnet_origin_y, int magnet_origin_z);

	// 初期値収束までのセンサ値格納
	void GetValue(int loop_number, double Sdata[]);

	// オフセット処理(ジャイロ)
	void OffsetGyro();

	// オフセット処理(地磁気)
	void OffsetMagnet();

	// 事前解析
	void PreAnalysis();

	// 姿勢推定時のセンサ値格納
	void GetValue(double Sdata[]);

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
	MatrixXf ToDCMtemp(VectorXf q);

	// 方向余弦行列(最終的に出力するもの)
	void ToDCM(VectorXf q);

	// 正規化
	VectorXf Normalize(VectorXf x);

	// カルマンゲインの修正
	void KalmanGainCorrect();

	// カルマンフィルタ(予測＋フィルタリング)
	void KalmanFilter(VectorXf y_value);

	// クォータニオン，共分散行列の適正値算出
	void Converge(int loop_number);

	// 姿勢推定
	void AtittudeEstimate();

	// 内部変数を返す
	VectorXf GetStateVariable();

	// DCNを返す
	MatrixXf GetDCM();

private:
	const float frequency = 0.01;
	const int stay = 1500; // 収束するまでに必要な静止状態のデータ数
	MatrixXf accel_array;
	MatrixXf gyro_array;
	MatrixXf magnet_array;

	VectorXf magnet_origin;

	float magnet_norm;
	float accel_norm;
	float inclination;

	MatrixXf y_array;

	int gamma; // 共分散行列の初期値

	MatrixXf DCM;


};

#endif