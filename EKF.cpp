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
	x = VectorXd(state_number);

	// �����ϐ��̏����l�̓T�u�N���X���Őݒ肷��

	// �����U�s��(�T�C�Y�̐錾�̂݁C�������̓T�u�N���X��)
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

}

// �����U�s��
MatrixXd EKF::Covariance(MatrixXd matrix)
{
	VectorXd average = matrix.rowwise().mean();
	MatrixXd cov1 = matrix.colwise() - average;
	MatrixXd cov2 = cov1.transpose();

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