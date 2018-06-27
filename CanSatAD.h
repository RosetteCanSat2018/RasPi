#ifndef _CANSATAD_H_
#define _CANSATAD_H_

#include "EKF.h"

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

	// ���K��
	VectorXf Normalize(VectorXf x);

	// �J���}���Q�C���̏C��
	void KalmanGainCorrect(MatrixXf& G, VectorXf y, const float accel_norm, const float inclination);

	// �����]���s��
	MatrixXf ToDCM(VectorXf q);

private:
	const float frequency = 0.01;
	const int stay = 15; // ��������܂łɕK�v�ȐÎ~��Ԃ̃f�[�^��
	MatrixXf accel_array;
	MatrixXf gyro_array;
	MatrixXf magnet_array;

	VectorXf magnet_origin;

	float magnet_norm;
	float accel_norm;
	float inclination;

	int gamma; // �����U�s��̏����l

	//MatrixXf DCM;


};

#endif