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
// calculate attitude and publish such that it can render something
// in rviz.

// Construct filter
AHRS u(1,1,1,1);

void adisCallback(const aauship::ADIS16405::ConstPtr& msg)
{
  /* Publist rviz information */
  static tf::TransformBroadcaster tfbc;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(-2,-2,-2) );
  tf::Quaternion q;
  q.setRPY(0,0,1);
  transform.setRotation(q);
  tfbc.sendTransform( tf::StampedTransform(transform, ros::Time::now(), "map", "bar"));

  /* AHRS update */
  u.calculateEulerAngles();
  u.MahonyAHRSupdate(msg->xgyro, msg->ygyro, msg->zgyro, msg->xaccl, msg->yaccl, msg->zaccl, msg->xmagn, msg->ymagn, msg->zmagn);
  ROS_INFO("Euler angles: [%.3f, %.3f, %.3f]", u.getEulerAngles(0), u.getEulerAngles(1), u.getEulerAngles(2));
}

// Used to configure the filter
void ahrsCallback(const aauship::ADIS16405::ConstPtr& msg)
{
//  setTuning(float kpA, float kiA, float kpM, float kiM);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ahrs_mahony");
  ros::NodeHandle adis;
  ros::Subscriber adissub = adis.subscribe("adis16405", 2, adisCallback);
  ros::NodeHandle ahrs;
  ros::Subscriber ahrssub = ahrs.subscribe("ahrs", 2, ahrsCallback);
  ros::spin();

  return 0;
}
