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
	x = VectorXd(state_number);

	// 内部変数の初期値はサブクラス内で設定する

	// 共分散行列(サイズの宣言のみ，初期化はサブクラスで)
	P = MatrixXd::Identity(state_number, state_number);

	// カルマンフィルタのパラメータ
	A = MatrixXd(state_number, state_number);
	B = MatrixXd(state_number, Qshape);
	x_ = VectorXd(state_number);
	P_ = MatrixXd(state_number, state_number);
	C = MatrixXd(y_number, state_number);
	y = VectorXd(y_number);
	G = MatrixXd(state_number, y_number);
	h_x = VectorXd(y_number);

}

// 共分散行列
MatrixXd EKF::Covariance(MatrixXd matrix)
{
	VectorXd average = matrix.rowwise().mean();
	MatrixXd cov1 = matrix.colwise() - average;
	MatrixXd cov2 = cov1.transpose();

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