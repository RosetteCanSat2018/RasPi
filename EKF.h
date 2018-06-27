/*******************************************
*
* 拡張カルマンフィルタの抽象クラス
*
*
* frequency : サンプリング周期[s]
* stay : 収束するまでに取得するデータ数
* accel_array : 加速度[m/s^2], shape=(3, stay)
* gyro_array : 角速度[rad/s], shape=(3, stay)
* magnet_array : 地磁気[], shape=(3, stay)
* y_array : システム観測値, shape=(9, stay)
* Q : 入力共分散行列, shape=(3, 3)
* R : 観測共分散行列, shape=(9, 9)
*
* x : 状態変数x^(k) shape=(7)
* y : 測定値y(k) shape=(9)
* P : 共分散行列P(k) shape=(7, 7)
*
* A : shape=(7, 7)
* B : shape=(7, 3)
* C : shape=(9, 7)
*
* x_ : 事前推定値x^-(k) shape=(7)
* P_ : 事前共分散行列P-(k) shape=(7, 7)
* G : カルマンゲインG(k) shape=(7, 9)
*******************************************/


#ifndef _EKF_H_
#define _EKF_H_

#include "Eigen/Core"
#include "Eigen/LU"

#define _USE_MATH_DEFINES
#include <math.h>

using namespace Eigen;

class EKF
{
public:
	// 使う変数のサイズを設定
	void SetParameter(int state_number, int Qshape, int y_number);

	// 内部変数と共分散行列の初期値は継承先のサブクラス内で定義する
	virtual void SetInitial(int gamma) = 0;

	//共分散行列を計算する関数
	MatrixXf Covariance(MatrixXf matrix);

	// 行列A
	virtual void MatrixA() = 0;

	// 行列B
	virtual void MatrixB() = 0;

	// 行列C
	virtual void MatrixC() = 0;

	// 事前共分散行列
	void PreCovariance();

	// カルマンゲイン
	void KalmanGain();

	// 関数h
	virtual void FunctionH() = 0;

	// カルマンフィルタ(予測＋フィルタリング)
	//virtual void KalmanFilter() = 0;

protected:
	
	int Qshape; // 入力共分散行列のサイズ

	// 初期設定のための変数
	int state_number; // 内部変数の数
	// int gamma; // 共分散行列の初期値

	int y_number; // 測定値の変数の数

	// 内部変数
	VectorXf x;
	// 共分散行列
	MatrixXf P;

	MatrixXf Q;
	MatrixXf R;	
	MatrixXf A;
	MatrixXf B;
	VectorXf x_;
	MatrixXf P_;
	MatrixXf C;
	VectorXf y;
	MatrixXf G;
	VectorXf h_x;

};

#endif

/*
MatrixXf *x = 0;

x = new MatrixXf(3, 3);
*x << 1, 1, 1, 1, 1, 1, 1, 1, 1;
PRINT_MAT(*x);
*/

/*
MatrixXf x;
x = MatrixXf(3, 3);
x << 1, 1, 1, 1, 1, 1, 1, 1, 1;
PRINT_MAT(x);
*/