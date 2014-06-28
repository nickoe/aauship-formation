#include "ros/ros.h"
#include "aauship/ADIS16405.h"
#include <tf/transform_broadcaster.h>
//extern "C" {
//	#include "aauship/MahonyAHRS.h"
//}
#include <aauship/test.h>
//#include "../include/test.h"
//
//
//
// This node shall read imu measurements and use an AHRS filter to
// calciulate attitude and publish such that it can render something
// in rviz.

// Construct filter
AHRS u(1,1,1,1);

void chatterCallback(const aauship::ADIS16405::ConstPtr& msg)
{
	static tf::TransformBroadcaster tfbc;
	tf::Transform transform;
	transform.setOrigin( tf::Vector3(-2,-2,-2) );
	tf::Quaternion q;
	q.setRPY(0,0,1);
	transform.setRotation(q);
	tfbc.sendTransform( tf::StampedTransform(transform, ros::Time::now(), "map", "bar"));

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ahrs_mahony");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("test", 1000, chatterCallback);
  ros::spin();

  return 0;
}
