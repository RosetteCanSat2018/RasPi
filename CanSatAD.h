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
	void SetParameter(int magnet_origin_x, int magnet_origin_y, int magnet_origin_z);

	// �����l�����܂ł̃Z���T�l�i�[
	void GetValue(int loop_number, double Sdata[]);

	// �I�t�Z�b�g����(�W���C��)
	void OffsetGyro();

	// �I�t�Z�b�g����(�n���C)
	void OffsetMagnet();

	// ���O���
	void PreAnalysis();

	// �p�����莞�̃Z���T�l�i�[
	void GetValue(double Sdata[]);

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
	MatrixXf ToDCMtemp(VectorXf q);

	// �����]���s��(�ŏI�I�ɏo�͂������)
	void ToDCM(VectorXf q);

	// ���K��
	VectorXf Normalize(VectorXf x);

	// �J���}���Q�C���̏C��
	void KalmanGainCorrect();

	// �J���}���t�B���^(�\���{�t�B���^�����O)
	void KalmanFilter(VectorXf y_value);

	// �N�H�[�^�j�I���C�����U�s��̓K���l�Z�o
	void Converge(int loop_number);

	// �p������
	void AtittudeEstimate();

	// �����ϐ���Ԃ�
	VectorXf GetStateVariable();

	// DCN��Ԃ�
	MatrixXf GetDCM();

private:
	const float frequency = 0.01;
	const int stay = 1500; // ��������܂łɕK�v�ȐÎ~��Ԃ̃f�[�^��
	MatrixXf accel_array;
	MatrixXf gyro_array;
	MatrixXf magnet_array;

	VectorXf magnet_origin;

	float magnet_norm;
	float accel_norm;
	float inclination;

	MatrixXf y_array;

	int gamma; // �����U�s��̏����l

	MatrixXf DCM;


};

#endif