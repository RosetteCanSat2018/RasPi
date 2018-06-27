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
	/// �ϐ��̐錾�������ōs�� //////////////////////////
	// �����ϐ�
	x = VectorXf(state_number);

	// �����ϐ��̏����l�̓T�u�N���X���Őݒ肷��

	// �����U�s��(�T�C�Y�̐錾�̂݁C�������̓T�u�N���X��)
	P = MatrixXf::Identity(state_number, state_number);

	// �J���}���t�B���^�̃p�����[�^
	A = MatrixXf(state_number, state_number);
	B = MatrixXf(state_number, Qshape);
	x_ = VectorXf(state_number);
	P_ = MatrixXf(state_number, state_number);
	C = MatrixXf(y_number, state_number);
	y = VectorXf(y_number);
	G = MatrixXf(state_number, y_number);
	h_x = VectorXf(y_number);

}

// �����U�s��
MatrixXf EKF::Covariance(MatrixXf matrix)
{
	VectorXf average = matrix.rowwise().mean();
	MatrixXf cov1 = matrix.colwise() - average;
	MatrixXf cov2 = cov1.transpose();

	return (cov1 * cov2) / matrix.cols();
}

// ���O�����U�s��
void EKF::PreCovariance()
{
	P_ =  A * P*A.transpose() + B * Q*B.transpose();
}

// �J���}���Q�C��
void EKF::KalmanGain()
{
	G = P_ * C.transpose() * (C * P_ * C.transpose() + R).inverse();
}