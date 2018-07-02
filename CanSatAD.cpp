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

// �X�[�p�[�N���X�̃I�[�o�[���C�h
void CanSatAD::SetParameter(double magnet_origin_x, double magnet_origin_y, double magnet_origin_z)
{

	//fopen_s(&fp, "test.csv", "w");
	fp2 = fopen("/home/pi/test.csv", "w");


	/// �ϐ��̐錾�������ōs�� //////////////////////////
	state_number = 7;
	Qshape = 3;
	y_number = 9;
	
	accel_array = MatrixXd(3, stay);
	gyro_array = MatrixXd(3, stay);
	magnet_array = MatrixXd(3, stay);

	gyro_average = VectorXd(3);

	magnet_origin = VectorXd(3);
	magnet_origin << magnet_origin_x, magnet_origin_y, magnet_origin_z;

	// �����ϐ�
	x = VectorXd(state_number);

	y_array = MatrixXd(y_number, stay);

	// �����U�s��(�T�C�Y�̐錾�̂�)
	P = MatrixXd::Identity(state_number, state_number);

	// �J���}���t�B���^�̃p�����[�^
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

// �����ϐ��̏����l�ݒ�
void CanSatAD::SetInitial(int gamma) {

	x << 0, 0, 0, 0, 0, 0, 1;

	P *= gamma;
}

// �����l�����܂ł̃Z���T�l�i�[(�����F���荞�݉񐔁C�Z���T�l�������Ă���z��)
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

// �I�t�Z�b�g����(�W���C��)
void CanSatAD::OffsetGyro()
{
	gyro_average = gyro_array.rowwise().mean();
	gyro_array.colwise() -= gyro_average;
}

// �I�t�Z�b�g����(�n���C)
void CanSatAD::OffsetMagnet()
{
	magnet_array.colwise() -= magnet_origin;
}

// �p�����莞�̃Z���T�l�i�[(�����I�t�Z�b�g��������)
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

// ���O���
void CanSatAD::PreAnalysis()
{
	// �Î~��Ԃ̒n���C�̑傫��
	VectorXd magnet_average = magnet_array.rowwise().mean();
	magnet_norm = magnet_average.norm();

	//�@�Î~��Ԃ̉����x�̑傫��
	VectorXd accel_average = accel_array.rowwise().mean();
	accel_norm = accel_average.norm();

	// ���p
	double inner = accel_average.dot(magnet_average); // ����
	double norm = magnet_norm * accel_norm; // �m�����̐�
	inclination = inner / norm; // cos��
	if (inclination < -1) { inclination = -1; } // -1<theta<1 �̔���
	if (inclination > 1) { inclination = 1; }
	inclination = acos(inclination) *(180 / M_PI); // [deg]
	if (inclination > 90) {
		inclination = 180 - inclination;
	}
	inclination = 90 - inclination;

	// ���͋����U�s��
	MatrixXd gyro_differential(3, stay - 1);
	for (int i = 0; i < stay - 1; i++) {
		gyro_differential(0, i) = (gyro_array(0, i + 1) - gyro_array(0, i)) / frequency;
		gyro_differential(1, i) = (gyro_array(1, i + 1) - gyro_array(1, i)) / frequency;
		gyro_differential(2, i) = (gyro_array(2, i + 1) - gyro_array(2, i)) / frequency;
	}
	Q = Covariance(gyro_differential);

	// �ϑ������U�s��
	y_array << gyro_array, magnet_array, accel_array;
	R = Covariance(y_array);
}

// �s��A
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

// �s��B
void CanSatAD::MatrixB()
{
	B << MatrixXd::Identity(3, 3), MatrixXd::Zero(4, 3);
}

// �s��C
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

// �֐�h
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

// �����]���s��(�֐�h���Ŏg������)
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

// ���K��
VectorXd CanSatAD::Normalize(VectorXd vector)
{
	vector.segment(3, 4) = vector.segment(3, 4).normalized();
	return vector;
}

// �J���}���Q�C���̏C��
void CanSatAD::KalmanGainCorrect(VectorXd y_value)
{
	// �����x�̑傫������
	VectorXd magnet_value(3);
	magnet_value = y_value.segment(3, 3);
	VectorXd accel_value(3);
	accel_value = y_value.segment(6, 3);
	bool norm_condition = abs((accel_value.norm() - accel_norm)) < (accel_norm*0.05);

	// �����x�̌�������
	double inner = magnet_value.dot(accel_value); // ����
	double norm = magnet_value.norm() * accel_value.norm(); // �m�����̐�
	double cos = inner / norm;
	if (cos < -1) { cos = -1; } // -1<theta<1 �̔���
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

// �J���}���t�B���^(�\���{�t�B���^�����O)
void CanSatAD::KalmanFilter(VectorXd y_value)
{
	/*** �\���X�e�b�v ********************************/
	MatrixA();

	MatrixB();

	x_ = Normalize(A*x);

	PreCovariance();

	MatrixC();

	/*** �t�B���^�����O�X�e�b�v **********************/
	KalmanGain();

	KalmanGainCorrect(y_value);

	FunctionH();

	x = x_ + G * (y_value - h_x);
	x = Normalize(x);
	
	P = (MatrixXd::Identity(7, 7) - G * C) * P_;
}

// �N�H�[�^�j�I���C�����U�s��̓K���l�Z�o
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

// �p������
void CanSatAD::AtittudeEstimate()

{
	KalmanFilter(y);
	
	ToDCM(x.segment(3, 4));

	//fprintf(fp2, "%f,%f,%f,%f,%f,%f,%f\n", x(0), x(1), x(2), x(3), x(4), x(5), x(6));
	fprintf(fp2, "%f,%f,%f,%f,%f,%f,%f,%f,%f\n", DCM(0, 0), DCM(0, 1), DCM(0, 2), DCM(1, 0), DCM(1, 1), DCM(1, 2), DCM(2, 0), DCM(2, 1), DCM(2, 2));
}

// �����ϐ���Ԃ�
VectorXd CanSatAD::GetStateVariable()
{
	return x;
}

// DCN��Ԃ�
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