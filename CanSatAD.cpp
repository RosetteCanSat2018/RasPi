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
	
	accel_array = MatrixXf(3, stay);
	gyro_array = MatrixXf(3, stay);
	magnet_array = MatrixXf(3, stay);

	magnet_origin = VectorXf(3);
	magnet_origin << magnet_origin_x, magnet_origin_y, magnet_origin_z;

	// 内部変数
	x = VectorXf(state_number);

	y_array = MatrixXf(y_number, stay);

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

	DCM = MatrixXf(3, 3);
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

// 事前解析
void CanSatAD::PreAnalysis()
{
	// 静止状態の地磁気の大きさ
	VectorXf magnet_average = magnet_array.rowwise().mean();
	magnet_norm = magnet_average.norm();

	//　静止状態の加速度の大きさ
	VectorXf accel_average = accel_array.rowwise().mean();
	accel_norm = accel_average.norm();

	// 伏角
	float inner = accel_average.dot(magnet_average); // 内積
	float norm = magnet_norm * accel_norm; // ノルムの積
	inclination = inner / norm; // cosθ
	if (inclination < -1) { inclination = -1; } // -1<theta<1 の判定
	if (inclination > 1) { inclination = 1; }
	inclination = acos(inclination) *(180 / M_PI); // [deg]
	if (inclination > 90) {
		inclination = 180 - inclination;
	}
	inclination = 90 - inclination;

	// 入力共分散行列
	MatrixXf gyro_differential(3, stay - 1);
	for (int i = 0; i < stay - 1; i++) {
		gyro_differential(0, i) = (gyro_array(0, i + 1) - gyro_array(0, i)) / frequency;
		gyro_differential(1, i) = (gyro_array(1, i + 1) - gyro_array(1, i)) / frequency;
		gyro_differential(2, i) = (gyro_array(2, i + 1) - gyro_array(2, i)) / frequency;
	}
	Q = Covariance(gyro_differential);

	// 観測共分散行列
	y_array << gyro_array, magnet_array, accel_array;
	R = Covariance(y_array);
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
	MatrixXf DCM_temp(3, 3);
	DCM_temp = ToDCMtemp(x_.segment(3, 4));

	float inclination_temp = inclination * (M_PI / 180);

	VectorXf matrix1 = VectorXf::Zero(3);
	matrix1(0) = magnet_norm * cos(inclination);
	matrix1(2) = magnet_norm * (-sin(inclination));
	matrix1 = DCM_temp * matrix1;

	VectorXf matrix2 = VectorXf::Zero(3);
	matrix2(2) = accel_norm;
	matrix2 = DCM_temp * matrix2;

	h_x << x_.segment(0, 3), matrix1, matrix2;
}

// 方向余弦行列(関数h内で使うもの)
MatrixXf CanSatAD::ToDCMtemp(VectorXf q)
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

	MatrixXf DCM_temp(3, 3);
	DCM_temp << C11, C12, C13, C21, C22, C23, C31, C32, C33;

	return DCM_temp;
}

void CanSatAD::ToDCM(VectorXf q)
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

	DCM << C11, C12, C13, C21, C22, C23, C31, C32, C33;
}

// 正規化
VectorXf CanSatAD::Normalize(VectorXf x)
{
	x.segment(3, 4) = x.segment(3, 4).normalized();
	return x;
}

// カルマンゲインの修正
void CanSatAD::KalmanGainCorrect()
{
	// 加速度の大きさ条件
	VectorXf magnet_value(3);
	magnet_value = y.segment(3, 3);
	VectorXf accel_value(3);
	accel_value = y.segment(6, 3);
	bool norm_condition = abs((accel_value.norm() - accel_norm)) < (accel_norm*0.05);

	// 加速度の向き条件
	float inner = magnet_value.dot(accel_value); // 内積
	float norm = magnet_value.norm() * accel_value.norm(); // ノルムの積
	float cos = inner / norm;
	if (cos < -1) { cos = -1; } // -1<theta<1 の判定
	if (cos > 1) { cos = 1; }
	float angle = acos(cos) *(180 / M_PI); // [deg]
	if (angle > 90) {
		angle = 180 - angle;
	}
	bool direction_condition = abs(angle - (90 - inclination)) < 5;

	if (!(norm_condition && direction_condition)) {
		G.block(0, 6, 7, 3) = MatrixXf::Zero(7, 3);
	}
}

// カルマンフィルタ(予測＋フィルタリング)
void CanSatAD::KalmanFilter(VectorXf y_value)
{
	/*** 予測ステップ ********************************/
	MatrixA();

	MatrixB();

	x_ = Normalize(x);

	PreCovariance();

	MatrixC();

	/*** フィルタリングステップ **********************/
	KalmanGain();

	KalmanGainCorrect();

	FunctionH();

	x = x_ + G * (y_value - h_x);

	P = (MatrixXf::Identity(7, 7) - G * C) * P_;
}

// クォータニオン，共分散行列の適正値算出
void CanSatAD::Converge(int loop_number)
{
	for (int i = 0; i < loop_number - 1; i++) {
		int k = i + 1;
		KalmanFilter(y_array.col(k));
	}
}

// 姿勢推定
void CanSatAD::AtittudeEstimate()
{
	KalmanFilter(y);
}

// 内部変数を返す
VectorXf CanSatAD::GetStateVariable()
{
	return x;
}

// DCNを返す
MatrixXf CanSatAD::GetDCM()
{
	return DCM;
}