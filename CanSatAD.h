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

	// �X�[�p�[�N���X�̃I�[�o�[���C�h
	void SetParameter(double magnet_origin_x, double magnet_origin_y, double magnet_origin_z);

	// �����l�����܂ł̃Z���T�l�i�[
	void GetValue(int loop_number, double ACGY[], double Mg[]);

	// �I�t�Z�b�g����(�W���C��)
	void OffsetGyro();

	// �I�t�Z�b�g����(�n���C)
	void OffsetMagnet();

	// ���O���
	void PreAnalysis();

	// �p�����莞�̃Z���T�l�i�[
	void GetValue(double ACGY[], double Mg[]);

	// �����ϐ��̏����l(gamma: �����U�s��̏����l)
	void SetInitial(int gamma);

	// �s��A
	void MatrixA();

	// �s��B
	void MatrixB();

	// �s��C
	void MatrixC();

	// �֐�h
	void FunctionH();

	// �����]���s��(�֐�h���Ŏg������)
	MatrixXd ToDCMtemp(VectorXd q);

	// �����]���s��(�ŏI�I�ɏo�͂������)
	void ToDCM(VectorXd q);

	// ���K��
	VectorXd Normalize(VectorXd x);

	// �J���}���Q�C���̏C��
	void KalmanGainCorrect(VectorXd y_value);

	// �J���}���t�B���^(�\���{�t�B���^�����O)
	void KalmanFilter(VectorXd y_value);

	// �N�H�[�^�j�I���C�����U�s��̓K���l�Z�o
	void Converge(int loop_number);

	// �p������
	void AtittudeEstimate();

	// �����ϐ���Ԃ�
	VectorXd GetStateVariable();

	// DCN��Ԃ�
	MatrixXd GetDCM();

	void Get();

	void Close();

private:
	const double frequency = 0.01;
	const int stay = 41728; // ��������܂łɕK�v�ȐÎ~��Ԃ̃f�[�^��
	MatrixXd accel_array;
	MatrixXd gyro_array;
	MatrixXd magnet_array;

	VectorXd gyro_average;

	VectorXd magnet_origin;

	double magnet_norm;
	double accel_norm;
	double inclination;

	MatrixXd y_array;

	int gamma; // �����U�s��̏����l

	MatrixXd DCM;

	FILE *fp2;


};

#endif