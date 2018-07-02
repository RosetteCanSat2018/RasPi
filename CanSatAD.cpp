#include <iostream>
#include "Eigen/Core"
#include "Eigen/LU"

#define _USE_MATH_DEFINES
#include <math.h>

#include "CanSatAD.h"

#include <stdio.h> 
#include <stdlib.h>

using namespace std;
using namespace Eigen;

// スーパークラスのオーバーライド
void CanSatAD::SetParameter(double magnet_origin_x, double magnet_origin_y, double magnet_origin_z)
{

	//fopen_s(&fp, "test.csv", "w");
	fp2 = fopen("/home/pi/test.csv", "w");


	/// 変数の宣言をここで行う //////////////////////////
	state_number = 7;
	Qshape = 3;
	y_number = 9;
	
	accel_array = MatrixXd(3, stay);
	gyro_array = MatrixXd(3, stay);
	magnet_array = MatrixXd(3, stay);

	gyro_average = VectorXd(3);

	magnet_origin = VectorXd(3);
	magnet_origin << magnet_origin_x, magnet_origin_y, magnet_origin_z;

	// 内部変数
	x = VectorXd(state_number);

	y_array = MatrixXd(y_number, stay);

	// 共分散行列(サイズの宣言のみ)
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

	DCM = MatrixXd(3, 3);
}

// 内部変数の初期値設定
void CanSatAD::SetInitial(int gamma) {

	x << 0, 0, 0, 0, 0, 0, 1;

	P *= gamma;
}

// 初期値収束までのセンサ値格納(引数：割り込み回数，センサ値が入っている配列)
void CanSatAD::GetValue(int loop_number, double ACGY[], double Mg[])
{
	accel_array(0, loop_number) = ACGY[0];
	accel_array(1, loop_number) = ACGY[1];
	accel_array(2, loop_number) = ACGY[2];
	gyro_array(0, loop_number) = ACGY[3] *(M_PI / 180);
	gyro_array(1, loop_number) = ACGY[4] *(M_PI / 180);
	gyro_array(2, loop_number) = ACGY[5] *(M_PI / 180);
	magnet_array(0, loop_number) = Mg[0];
	magnet_array(1, loop_number) = Mg[1];
	magnet_array(2, loop_number) = Mg[2];
}

// オフセット処理(ジャイロ)
void CanSatAD::OffsetGyro()
{
	gyro_average = gyro_array.rowwise().mean();
	gyro_array.colwise() -= gyro_average;
}

// オフセット処理(地磁気)
void CanSatAD::OffsetMagnet()
{
	magnet_array.colwise() -= magnet_origin;
}

// 姿勢推定時のセンサ値格納(★★オフセットもここで)
void CanSatAD::GetValue(double ACGY[], double Mg[])
{
	y(0) = ACGY[3] *(M_PI / 180) - gyro_average(0); // gyro
	y(1) = ACGY[4] *(M_PI / 180) - gyro_average(1);
	y(2) = ACGY[5] *(M_PI / 180) - gyro_average(2);
	y(3) = Mg[0] - magnet_origin(0); // magnet
	y(4) = Mg[1] - magnet_origin(1);
	y(5) = Mg[2] - magnet_origin(2);
	y(6) = ACGY[0]; // accel
	y(7) = ACGY[1];
	y(8) = ACGY[2];
}

// 事前解析
void CanSatAD::PreAnalysis()
{
	// 静止状態の地磁気の大きさ
	VectorXd magnet_average = magnet_array.rowwise().mean();
	magnet_norm = magnet_average.norm();

	//　静止状態の加速度の大きさ
	VectorXd accel_average = accel_array.rowwise().mean();
	accel_norm = accel_average.norm();

	// 伏角
	double inner = accel_average.dot(magnet_average); // 内積
	double norm = magnet_norm * accel_norm; // ノルムの積
	inclination = inner / norm; // cosθ
	if (inclination < -1) { inclination = -1; } // -1<theta<1 の判定
	if (inclination > 1) { inclination = 1; }
	inclination = acos(inclination) *(180 / M_PI); // [deg]
	if (inclination > 90) {
		inclination = 180 - inclination;
	}
	inclination = 90 - inclination;

	// 入力共分散行列
	MatrixXd gyro_differential(3, stay - 1);
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
	A = MatrixXd::Identity(state_number, state_number);
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
	B << MatrixXd::Identity(3, 3), MatrixXd::Zero(4, 3);
}

// 行列C
void CanSatAD::MatrixC()
{
	double inclination_temp = inclination * (M_PI / 180);
	MatrixXd matrix0 = MatrixXd::Zero(6, 1);
	matrix0(0, 0) = magnet_norm * cos(inclination_temp) * 2;
	matrix0(2, 0) = magnet_norm * sin(inclination_temp) * (-2);
	matrix0(5, 0) = accel_norm * 2;

	VectorXd q(4);
	q = x_.segment(3, 4);

	MatrixXd matrix1 = MatrixXd::Zero(6, 6);
	MatrixXd matrix2 = MatrixXd::Zero(6, 6);
	MatrixXd matrix3 = MatrixXd::Zero(6, 6);
	MatrixXd matrix4 = MatrixXd::Zero(6, 6);

	MatrixXd matrix1_1(3, 3);
	matrix1_1 << q(0), q(1), q(2), q(1), -q(0), q(3), q(2), -q(3), -q(0);
	matrix1.block(0, 0, 3, 3) = matrix1_1;
	matrix1.block(3, 3, 3, 3) = matrix1_1;

	MatrixXd matrix2_1(3, 3);
	matrix2_1 << -q(1), q(0), -q(3), q(0), q(1), q(2), q(3), q(2), -q(1);
	matrix2.block(0, 0, 3, 3) = matrix2_1;
	matrix2.block(3, 3, 3, 3) = matrix2_1;

	MatrixXd matrix3_1(3, 3);
	matrix3_1 << -q(2), q(3), q(0), -q(3), -q(2), q(1), q(0), q(1), q(2);
	matrix3.block(0, 0, 3, 3) = matrix3_1;
	matrix3.block(3, 3, 3, 3) = matrix3_1;

	MatrixXd matrix4_1(3, 3);
	matrix4_1 << q(3), q(2), -q(1), -q(2), q(3), q(0), q(1), -q(0), q(3);
	matrix4.block(0, 0, 3, 3) = matrix4_1;
	matrix4.block(3, 3, 3, 3) = matrix4_1;

	matrix1 = matrix1 * matrix0;
	matrix2 = matrix2 * matrix0;
	matrix3 = matrix3 * matrix0;
	matrix4 = matrix4 * matrix0;

	MatrixXd matrix(6, 4);
	matrix << matrix1, matrix2, matrix3, matrix4;

	C = MatrixXd::Identity(y_number, state_number);
	C.block(3, 3, 6, 4) = matrix;
}

// 関数h
void CanSatAD::FunctionH()
{
	MatrixXd DCM_temp(3, 3);
	DCM_temp = ToDCMtemp(x_.segment(3, 4));

	double inclination_temp = inclination * (M_PI / 180);

	VectorXd matrix1 = VectorXd::Zero(3);
	matrix1(0) = magnet_norm * cos(inclination_temp);
	matrix1(2) = magnet_norm * (-sin(inclination_temp));
	matrix1 = DCM_temp * matrix1;

	VectorXd matrix2 = VectorXd::Zero(3);
	matrix2(2) = accel_norm;
	matrix2 = DCM_temp * matrix2;

	h_x << x_.segment(0, 3), matrix1, matrix2;
}

// 方向余弦行列(関数h内で使うもの)
MatrixXd CanSatAD::ToDCMtemp(VectorXd q)
{
	double C11 = q(0) * q(0) - q(1) * q(1) - q(2) * q(2) + q(3) * q(3);
	double C12 = 2 * (q(0) * q(1) + q(2) * q(3));
	double C13 = 2 * (q(2) * q(0) - q(1) * q(3));
	double C21 = 2 * (q(0) * q(1) - q(2) * q(3));
	double C22 = q(1) * q(1) - q(2) * q(2) - q(0) * q(0) + q(3) * q(3);
	double C23 = 2 * (q(1) * q(2) + q(0) * q(3));
	double C31 = 2 * (q(2) * q(0) + q(1) * q(3));
	double C32 = 2 * (q(1) * q(2) - q(0) * q(3));
	double C33 = q(2) * q(2) - q(0) * q(0) - q(1) * q(1) + q(3) * q(3);

	MatrixXd DCM_temp(3, 3);
	DCM_temp << C11, C12, C13, C21, C22, C23, C31, C32, C33;

	return DCM_temp;
}

void CanSatAD::ToDCM(VectorXd q)
{
	double C11 = q(0) * q(0) - q(1) * q(1) - q(2) * q(2) + q(3) * q(3);
	double C12 = 2 * (q(0) * q(1) + q(2) * q(3));
	double C13 = 2 * (q(2) * q(0) - q(1) * q(3));
	double C21 = 2 * (q(0) * q(1) - q(2) * q(3));
	double C22 = q(1) * q(1) - q(2) * q(2) - q(0) * q(0) + q(3) * q(3);
	double C23 = 2 * (q(1) * q(2) + q(0) * q(3));
	double C31 = 2 * (q(2) * q(0) + q(1) * q(3));
	double C32 = 2 * (q(1) * q(2) - q(0) * q(3));
	double C33 = q(2) * q(2) - q(0) * q(0) - q(1) * q(1) + q(3) * q(3);

	DCM << C11, C12, C13, C21, C22, C23, C31, C32, C33;
}

// 正規化
VectorXd CanSatAD::Normalize(VectorXd vector)
{
	vector.segment(3, 4) = vector.segment(3, 4).normalized();
	return vector;
}

// カルマンゲインの修正
void CanSatAD::KalmanGainCorrect(VectorXd y_value)
{
	// 加速度の大きさ条件
	VectorXd magnet_value(3);
	magnet_value = y_value.segment(3, 3);
	VectorXd accel_value(3);
	accel_value = y_value.segment(6, 3);
	bool norm_condition = abs((accel_value.norm() - accel_norm)) < (accel_norm*0.05);

	// 加速度の向き条件
	double inner = magnet_value.dot(accel_value); // 内積
	double norm = magnet_value.norm() * accel_value.norm(); // ノルムの積
	double cos = inner / norm;
	if (cos < -1) { cos = -1; } // -1<theta<1 の判定
	if (cos > 1) { cos = 1; }
	double angle = acos(cos) *(180 / M_PI); // [deg]
	if (angle > 90) {
		angle = 180 - angle;
	}
	bool direction_condition = abs(angle - (90 - inclination)) < 5;

	if (!(norm_condition && direction_condition)) {
		G.block(0, 6, 7, 3) = MatrixXd::Zero(7, 3);
	}
}

// カルマンフィルタ(予測＋フィルタリング)
void CanSatAD::KalmanFilter(VectorXd y_value)
{
	/*** 予測ステップ ********************************/
	MatrixA();

	MatrixB();

	x_ = Normalize(A*x);

	PreCovariance();

	MatrixC();

	/*** フィルタリングステップ **********************/
	KalmanGain();

	KalmanGainCorrect(y_value);

	FunctionH();

	x = x_ + G * (y_value - h_x);
	x = Normalize(x);
	
	P = (MatrixXd::Identity(7, 7) - G * C) * P_;
}

// クォータニオン，共分散行列の適正値算出
void CanSatAD::Converge(int loop_number)
{
	for (int i = 0; i < loop_number - 1; i++) {
		int k = i + 1;
		KalmanFilter(y_array.col(k));
		
		ToDCM(x.segment(3, 4));

		//fprintf(fp2, "%f,%f,%f,%f,%f,%f,%f\n", x(0),x(1),x(2),x(3),x(4),x(5),x(6));
		fprintf(fp2, "%f,%f,%f,%f,%f,%f,%f,%f,%f\n", DCM(0,0), DCM(0,1), DCM(0,2), DCM(1,0), DCM(1,1), DCM(1,2), DCM(2,0), DCM(2,1), DCM(2,2));
	}
}

// 姿勢推定
void CanSatAD::AtittudeEstimate()

{
	KalmanFilter(y);
	
	ToDCM(x.segment(3, 4));

	//fprintf(fp2, "%f,%f,%f,%f,%f,%f,%f\n", x(0), x(1), x(2), x(3), x(4), x(5), x(6));
	fprintf(fp2, "%f,%f,%f,%f,%f,%f,%f,%f,%f\n", DCM(0, 0), DCM(0, 1), DCM(0, 2), DCM(1, 0), DCM(1, 1), DCM(1, 2), DCM(2, 0), DCM(2, 1), DCM(2, 2));
}

// 内部変数を返す
VectorXd CanSatAD::GetStateVariable()
{
	return x;
}

// DCNを返す
MatrixXd CanSatAD::GetDCM()
{
	return DCM;
}

void CanSatAD::Get()
{
	/*cout << "accel_norm = " << accel_norm << endl << endl;
	cout << "magnet_norm = " << magnet_norm << endl << endl;
	cout << "inlicimation = " << inclination << endl << endl;
	cout << "Q = " << endl << Q << endl << endl;
	cout << "R = " << endl << R << endl << endl;*/
	cout << "A = " << endl << A << endl << endl;
	//cout << "B = " << endl << B << endl << endl;
	cout << "C = " << endl << C << endl << endl;
	cout << "h_x = " << endl << h_x << endl << endl;
	cout << "G = " << endl << G << endl << endl;
	cout << "P_ = " << endl << P_ << endl << endl;
	cout << "x = " << endl << x << endl << endl;
}

void CanSatAD::Close()
{
	fclose(fp);
}