#include <iostream>
#include "Eigen/Core"
#include "Eigen/LU"

#define _USE_MATH_DEFINES
#include <math.h>

#include "CanSatAD.h"

using namespace std;
using namespace Eigen;

// スーパークラスのオーバーライド
void CanSatAD::SetParameter(int magnet_origin_x, int magnet_origin_y, int magnet_origin_z)
{
	/// 変数の宣言をここで行う //////////////////////////
	state_number = 7;
	Qshape = 3;
	y_number = 9;

	accel_array(3, stay);
	gyro_array(3, stay);
	magnet_array(3, stay);

	magnet_origin(3);
	magnet_origin << magnet_origin_x, magnet_origin_y, magnet_origin_z;

	// 内部変数
	x = VectorXf(state_number);


	// 共分散行列(サイズの宣言のみ)
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

	//DCM = MatrixXf(3, 3);
}

// 内部変数の初期値設定
void CanSatAD::SetInitial(int gamma) {

	x << 0, 0, 0, 0, 0, 0, 1;

	P *= gamma;
}

// 初期値収束までのセンサ値格納(引数：割り込み回数，センサ値が入っている配列)
void CanSatAD::GetValue(int loop_number, double Sdata[])
{
	accel_array(0, loop_number) = Sdata[0];
	accel_array(1, loop_number) = Sdata[1];
	accel_array(2, loop_number) = Sdata[2];
	gyro_array(0, loop_number) = Sdata[3];
	gyro_array(1, loop_number) = Sdata[4];
	gyro_array(2, loop_number) = Sdata[5];
	magnet_array(0, loop_number) = Sdata[6];
	magnet_array(1, loop_number) = Sdata[7];
	magnet_array(2, loop_number) = Sdata[8];
}

// オフセット処理(ジャイロ)
void CanSatAD::OffsetGyro()
{
	VectorXf average = gyro_array.rowwise().mean();
	gyro_array.colwise() -= average;
}

// 姿勢推定時のセンサ値格納
void CanSatAD::GetValue(double Sdata[])
{
	y(0) = Sdata[3]; // gyro
	y(1) = Sdata[4];
	y(2) = Sdata[5];
	y(3) = Sdata[6]; // magnet
	y(4) = Sdata[7];
	y(5) = Sdata[8];
	y(6) = Sdata[0]; // accel
	y(7) = Sdata[1];
	y(8) = Sdata[2];
}

// オフセット処理(地磁気)
void CanSatAD::OffsetMagnet()
{
	magnet_array.colwise() -= magnet_origin;
}

// 行列A
void CanSatAD::MatrixA()
{
	A = MatrixXf::Identity(state_number, state_number);
	A(3, 0) = x(6) * frequency / 2;
	A(4, 0) = x(5) * frequency / 2;
	A(5, 0) = -x(4) * frequency / 2;
	A(6, 0) = -x(3) * frequency / 2;
	A(3, 1) = -x(5) * frequency / 2;
	A(4, 1) = x(6) * frequency / 2;
	A(5, 1) = x(3) * frequency / 2;
	A(6, 1) = -x(4) * frequency / 2;
	A(3, 2) = x(4) * frequency / 2;
	A(4, 2) = -x(3) * frequency / 2;
	A(5, 2) = x(6) * frequency / 2;
	A(6, 2) = -x(5) * frequency / 2;
}

// 行列B
void CanSatAD::MatrixB()
{
	B << MatrixXf::Identity(3, 3), MatrixXf::Zero(4, 3);
}

// 行列C
void CanSatAD::MatrixC()
{
	float inclination_temp = inclination * (M_PI / 180);
	MatrixXf matrix0 = MatrixXf::Zero(6, 1);
	matrix0(0, 0) = magnet_norm * cos(inclination_temp) * 2;
	matrix0(2, 0) = magnet_norm * sin(inclination_temp) * (-2);
	matrix0(5, 0) = accel_norm * 2;

	VectorXf q(4);
	q = x.segment(3, 4);

	MatrixXf matrix1 = MatrixXf::Zero(6, 6);
	MatrixXf matrix2 = MatrixXf::Zero(6, 6);
	MatrixXf matrix3 = MatrixXf::Zero(6, 6);
	MatrixXf matrix4 = MatrixXf::Zero(6, 6);

	MatrixXf matrix1_1(3, 3);
	matrix1_1 << q(0), q(1), q(2), q(1), -q(0), q(3), q(2), -q(3), -q(0);
	matrix1.block(0, 0, 3, 3) = matrix1_1;
	matrix1.block(3, 3, 3, 3) = matrix1_1;

	MatrixXf matrix2_1(3, 3);
	matrix2_1 << -q(1), q(0), -q(3), q(0), q(1), q(2), q(3), q(2), -q(1);
	matrix2.block(0, 0, 3, 3) = matrix2_1;
	matrix2.block(3, 3, 3, 3) = matrix2_1;

	MatrixXf matrix3_1(3, 3);
	matrix3_1 << -q(2), q(3), q(0), -q(3), -q(2), q(1), q(0), q(1), q(2);
	matrix3.block(0, 0, 3, 3) = matrix3_1;
	matrix3.block(3, 3, 3, 3) = matrix3_1;

	MatrixXf matrix4_1(3, 3);
	matrix4_1 << q(3), q(2), -q(1), -q(2), q(3), q(0), q(1), -q(0), q(3);
	matrix4.block(0, 0, 3, 3) = matrix4_1;
	matrix4.block(3, 3, 3, 3) = matrix4_1;

	matrix1 = matrix1 * matrix0;
	matrix2 = matrix2 * matrix0;
	matrix3 = matrix3 * matrix0;
	matrix4 = matrix4 * matrix0;

	MatrixXf matrix(6, 4);
	matrix << matrix1, matrix2, matrix3, matrix4;

	C = MatrixXf::Identity(y_number, state_number);
	C.block(3, 3, 6, 4) = matrix;
}

// 関数h
void CanSatAD::FunctionH()
{
	MatrixXf DCM(3, 3);
	DCM = ToDCM(x_.segment(3, 4));

	float inclination_temp = inclination * (M_PI / 180);

	VectorXf matrix1 = VectorXf::Zero(3);
	matrix1(0) = magnet_norm * cos(inclination);
	matrix1(2) = magnet_norm * (-sin(inclination));
	matrix1 = DCM * matrix1;

	VectorXf matrix2 = VectorXf::Zero(3);
	matrix2(2) = accel_norm;
	matrix2 = DCM * matrix2;

	VectorXf h_x(9);
	h_x << x_.segment(0, 3), matrix1, matrix2;
}

// 正規化
VectorXf Normalize(VectorXf x)
{

}

// カルマンゲインの修正
void KalmanGainCorrect(MatrixXf& G, VectorXf y, const float accel_norm, const float inclination)
{

}

// 方向余弦行列
MatrixXf ToDCM(VectorXf q)
{
	float C11 = q(0) * q(0) - q(1) * q(1) - q(2) * q(2) + q(3) * q(3);
	float C12 = 2 * (q(0) * q(1) + q(2) * q(3));
	float C13 = 2 * (q(2) * q(0) - q(1) * q(3));
	float C21 = 2 * (q(0) * q(1) - q(2) * q(3));
	float C22 = q(1) * q(1) - q(2) * q(2) - q(0) * q(0) + q(3) * q(3);
	float C23 = 2 * (q(1) * q(2) + q(0) * q(3));
	float C31 = 2 * (q(2) * q(0) + q(1) * q(3));
	float C32 = 2 * (q(1) * q(2) - q(0) * q(3));
	float C33 = q(2) * q(2) - q(0) * q(0) - q(1) * q(1) + q(3) * q(3);

	MatrixXf DCM(3, 3);
	DCM << C11, C12, C13, C21, C22, C23, C31, C32, C33;

	return DCM;
}