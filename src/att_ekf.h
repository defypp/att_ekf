#include <iostream>
#include <Eigen/Eigen>
#include "conversion.h"

using namespace std;
using namespace Eigen;
class Att_ekf
{
public:
	Att_ekf();
	~Att_ekf();
	void predict(double t);
	void update_magnetic(Vector3d& mag, double t);
	void update_imu(Vector3d &acc, Vector3d & gyro, double t);
	Matrix3d get_rotation_matrix();

private:
	double curr_t;
	Matrix<double, 12, 1> x;//anguler velocity, angular acceleration velocity, gravity field, magnetic field 
	Matrix<double, 12, 12> P;//covariance
	Matrix<double, 12, 12> Q;//process noise

	Matrix<double, 6, 6> R_imu;//measurement noise
	Matrix<double, 3, 3> R_mag;//measurement noise

	bool imu_initialized;
	bool mag_initialized;
};
