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
}
Att_ekf::~Att_ekf()
{

}
void Att_ekf::predict(double t)
{
	double dt = t - dt;
	Vector3d w = x.head(3);
	Vector3d ra = x.segment<3>(6);
	Vector3d rm = x.tail(3);

	Matrix3d skew_w = skew_symmetric(w);
	x.segment<3>(0) = x.segment<3>(0) + x.segment<3>(3)*dt;
	x.segment<3>(6) = x.segment<3>(6) + skew_w*x.segment<3>(6)*dt;
	x.segment<3>(9) = x.segment<3>(9) + skew_w*x.segment<3>(9)*dt;

	Matrix<double, 12, 12> A = MatrixXd::Identity(12, 12);
	A.block<3, 3>(0, 3) += MatrixXd::Identity(3, 3)*dt;
	A.block<3, 3>(2*3, 2*3) += -skew_symmetric(ra)*dt;
	A.block<3, 3>(3*3, 3*3) += -skew_symmetric(rm)*dt;

	P = A*P*A.transpose() + Q;
	
}
void Att_ekf::update_magnetic(Vector3d& mag, double t)
{
	if(!mag_initialized)
	{
		x.tail(3) = mag;
		mag_initialized = true;
		curr_t = t;
		return;
	}
	predict(t);



}
void Att_ekf::update_imu(Vector3d &acc, Vector3d & gyro, double t)
{
	if(!imu_initialized)
	{
		x.head(3) = gyro;
		x.segment<3>(6) = acc;
		imu_initialized = true;
		curr_t = t;
		return;
	}
	predict(t);
}


Matrix3d Att_ekf::get_rotation_matrix()
{
	Matrix3d Rbn;//NED to body
	Vector3d ra = x.segment<3>(6);
	Vector3d rm = x.segment<3>(9);
	Vector3d Iz = -ra/ra.norm();
	Vector3d Iy = skew_symmetric(Iz)*rm;
	Iy /= Iy.norm();
	Vector3d Ix = skew_symmetric(Iy)*Iz;
	Rbn.col(0) = Ix; Rbn.col(1) = Iy; Rbn.col(2) = Iz;
	return Rbn;
}