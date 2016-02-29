#include "att_ekf.h"
#include <iostream>
#include <Eigen/Eigen>
using namespace std;
using namespace Eigen;

Matrix3d skew_symmetric(Vector3d& v)
{
	Matrix3d m;
	m << 0, -v(2), v(1),
	     v(2), 0, -v(0),
	     -v(1), v(0), 0;
    return m;
}

Att_ekf::Att_ekf()
{
	x.setZero();
	imu_initialized = false;
	mag_initialized = false;

	R_imu.setZero();
	R_imu.block<3, 3>(0, 0) = R_gyro;
	R_imu.block<3, 3>(3, 3) = R_acc;

	P.setZero();
	P.block<3, 3>(0, 0) = R_gyro;
	P.block<3, 3>(3, 3) = R_gyro;
	P.block<3, 3>(6, 6) = R_acc;
	P.block<3, 3>(9, 9) = R_mag;
}
Att_ekf::~Att_ekf()
{

}
void Att_ekf::predict(double t)
{
	double dt = t - curr_t;
	//cout << "dt :" << dt << endl;
	Vector3d w = x.head(3);
	Vector3d wa  = x.segment<3>(3);
	Vector3d ra = x.segment<3>(6);
	Vector3d rm = x.tail(3);

	x.segment<3>(0) += wa*dt;
	x.segment<3>(6) += -skew_symmetric(w)*ra*dt;
	x.segment<3>(9) += -skew_symmetric(w)*rm*dt;

	Matrix<double, 12, 12> A = MatrixXd::Identity(12, 12);
	A.block<3, 3>(0, 3) += MatrixXd::Identity(3, 3)*dt;
	A.block<3, 3>(6, 6) += -skew_symmetric(w)*dt;
	A.block<3, 3>(9, 9) += -skew_symmetric(w)*dt;
	A.block<3, 3>(6, 0) += skew_symmetric(ra)*dt;
	A.block<3, 3>(9, 0) += skew_symmetric(rm)*dt;

	P = A*P*A.transpose() + Q;//Q?
	curr_t = t;
	
}
void Att_ekf::update_magnetic(Vector3d& mag, double t)
{
	if(!(mag_initialized && imu_initialized) )
	{
		x.tail(3) = mag;
		mag_initialized = true;
		curr_t = t;
		return;
	}
	if(t < curr_t) 
	{
		cout << "t is smaller than curr_t" << endl;
		return;
	}
	predict(t);

	MatrixXd H = MatrixXd::Zero(3, 12);
	VectorXd z = mag;
	H.block<3, 3>(0, 9) = Matrix3d::Identity();

	MatrixXd K = P.inverse()*H.transpose()*(H*P.inverse()*H.transpose() + R_mag).inverse();
	MatrixXd I = MatrixXd::Identity(12, 12);

	x = x + K*(z - H*x);
	P = (I- K*H)*P.inverse();
	//cout << "update mag: " << x.transpose() << endl;
}
void Att_ekf::update_imu(Vector3d &acc, Vector3d & gyro, double t)
{
	if(!(mag_initialized && imu_initialized))
	{
		x.head(3) = gyro;
		x.segment<3>(6) = acc;
		imu_initialized = true;
		cout << "imu initialize: " << x.transpose() << endl;
		curr_t = t;
		return;
	}
	if(t < curr_t) 
	{
		cout << "t is smaller than curr_t" << endl;
		return;
	}
	predict(t);
	
	MatrixXd H = MatrixXd::Zero(6, 12);
	VectorXd z(6);
	z.head(3) = gyro;
	z.tail(3) = acc;
	H.block<3, 3>(0, 0) = Matrix3d::Identity();
	H.block<3, 3>(3, 6) = Matrix3d::Identity();

	MatrixXd K = P.inverse()*H.transpose()*(H*P.inverse()*H.transpose() + R_imu).inverse();
	MatrixXd I = MatrixXd::Identity(12, 12);

	x = x + K*(z - H*x);
	P = (I- K*H)*P.inverse();
	//cout << "update imu: " << x.transpose() << endl;
}


Matrix3d Att_ekf::get_rotation_matrix()
{
	if(!(mag_initialized && imu_initialized)) return Matrix3d::Identity();
	Matrix3d Rbn;//ENU to body
	Vector3d ra = x.segment<3>(6);
	Vector3d rm = x.segment<3>(9);
	Vector3d Iz = ra;//ENU coordinate
	Vector3d Iy = skew_symmetric(Iz)*rm;//why rm in ENU coordinate is [1 0 x], it should be [0 1 x]
	Vector3d Ix = skew_symmetric(Iy)*Iz;
	Ix /= Ix.norm(); Iy /= Iy.norm(); Iz /= Iz.norm();
	Rbn.col(0) = Ix; Rbn.col(1) = Iy; Rbn.col(2) = Iz;
	return Rbn.transpose();
}