/*******************************************
*
* �g���J���}���t�B���^�̒��ۃN���X
*
*
* frequency : �T���v�����O����[s]
* stay : ��������܂łɎ擾����f�[�^��
* accel_array : �����x[m/s^2], shape=(3, stay)
* gyro_array : �p���x[rad/s], shape=(3, stay)
* magnet_array : �n���C[], shape=(3, stay)
* y_array : �V�X�e���ϑ��l, shape=(9, stay)
* Q : ���͋����U�s��, shape=(3, 3)
* R : �ϑ������U�s��, shape=(9, 9)
*
* x : ��ԕϐ�x^(k) shape=(7)
* y : ����ly(k) shape=(9)
* P : �����U�s��P(k) shape=(7, 7)
*
* A : shape=(7, 7)
* B : shape=(7, 3)
* C : shape=(9, 7)
*
* x_ : ���O����lx^-(k) shape=(7)
* P_ : ���O�����U�s��P-(k) shape=(7, 7)
* G : �J���}���Q�C��G(k) shape=(7, 9)
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
	// �g���ϐ��̃T�C�Y��ݒ�
	void SetParameter(int state_number, int Qshape, int y_number);

	// �����ϐ��Ƌ����U�s��̏����l�͌p����̃T�u�N���X���Œ�`����
	virtual void SetInitial(int gamma) = 0;

	//�����U�s����v�Z����֐�
	MatrixXf Covariance(MatrixXf matrix);

	// �s��A
	virtual void MatrixA() = 0;

	// �s��B
	virtual void MatrixB() = 0;

	// �s��C
	virtual void MatrixC() = 0;

	// ���O�����U�s��
	void PreCovariance();

	// �J���}���Q�C��
	void KalmanGain();

	// �֐�h
	virtual void FunctionH() = 0;

	// �J���}���t�B���^(�\���{�t�B���^�����O)
	//virtual void KalmanFilter() = 0;

protected:
	
	int Qshape; // ���͋����U�s��̃T�C�Y

	// �����ݒ�̂��߂̕ϐ�
	int state_number; // �����ϐ��̐�
	// int gamma; // �����U�s��̏����l

	int y_number; // ����l�̕ϐ��̐�

	// �����ϐ�
	VectorXf x;
	// �����U�s��
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