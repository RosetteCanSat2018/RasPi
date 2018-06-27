#include <iostream>
#include "Eigen/Core"
#include "Eigen/LU"

#define _USE_MATH_DEFINES
#include <math.h>

#include "EKF.h"

using namespace std;
using namespace Eigen;

 
void EKF::SetParameter(int state_number, int Qshape, int y_number)
{
	/// 変数の宣言をここで行う //////////////////////////
	// 内部変数
	x = VectorXf(state_number);

	// 内部変数の初期値はサブクラス内で設定する

	// 共分散行列(サイズの宣言のみ，初期化はサブクラスで)
	P = MatrixXf::Identity(state_number, state_number);

	// カルマンフィルタのパラメータ
	A = MatrixXf(state_number, state_number);
	B = MatrixXf(state_number, Qshape);
	x_ = VectorXf(state_number);
	P_ = MatrixXf(state_number, state_number);
	C = MatrixXf(y_number, state_number);
	y = VectorXf(y_number);
	G = MatrixXf(state_number, y_number);
	h_x = VectorXf(y_number);

}

// 共分散行列
MatrixXf EKF::Covariance(MatrixXf matrix)
{
	VectorXf average = matrix.rowwise().mean();
	MatrixXf cov1 = matrix.colwise() - average;
	MatrixXf cov2 = cov1.transpose();

	return (cov1 * cov2) / matrix.cols();
}

// 事前共分散行列
void EKF::PreCovariance()
{
	P_ =  A * P*A.transpose() + B * Q*B.transpose();
}

// カルマンゲイン
void EKF::KalmanGain()
{
	G = P_ * C.transpose() * (C * P_ * C.transpose() + R).inverse();
}