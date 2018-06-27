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
	MatrixXf PreCovariance();

	// �J���}���Q�C��
	MatrixXf KalmanGain();

	// �֐�h
	virtual void FunctionH() = 0;

	// �J���}���t�B���^(�\���{�t�B���^�����O)
	void KalmanFilter(MatrixXf A, MatrixXf B, VectorXf x_, MatrixXf P_, MatrixXf C, VectorXf y, MatrixXf G, MatrixXf DCM, VectorXf h_x, VectorXf x, MatrixXf P, MatrixXf Q, MatrixXf R, float magnet_norm, float accel_norm, float inclination);

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