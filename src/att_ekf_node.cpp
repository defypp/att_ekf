#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <tf/transform_listener.h>

#include <Eigen/Eigen>
#include "att_ekf.h"
#include "conversion.h"

using namespace std;
using namespace Eigen;


void magCallback(const sensor_msgs::MagneticFieldConstPtr& msg)
{
	Vector3d mag;
	mag(0) = msg->magnetic_field.x;
	mag(1) = msg->magnetic_field.y;
	mag(2) = msg->magnetic_field.z;
	cout << "mag: " << mag.transpose() << endl;
}


void imuCallback(const sensor_msgs::ImuConstPtr & msg)
{
	Quaterniond q;
	q.w() = msg->orientation.w;
	q.x() = msg->orientation.x;
	q.y() = msg->orientation.y;
	q.z() = msg->orientation.z;
	cout << "q: " << q.w() << " " << q.vec().transpose() << endl;
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "laser_localization");
	ros::NodeHandle n("~");
	ros::Subscriber imu_sub = n.subscribe("/imu", 100, imuCallback);
	ros::Subscriber mag_sub = n.subscribe("/magnetic_field", 100, magCallback);

	ros::spin();

	return 0;
}