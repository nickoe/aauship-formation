#include "ros/ros.h"
#include "aauship/ADIS16405.h"
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
//extern "C" {
//	#include "aauship/MahonyAHRS.h"
//}
#include <aauship/test.h>

//
// This node shall read imu measurements and use an AHRS filter to
// calculate attitude and publish such that it can render something
// in rviz.

// Construct filter
AHRS u(8.8,0.5,8.8,0.5);

void adisCallback(const aauship::ADIS16405::ConstPtr& msg)
{
  /* AHRS update */
  u.calculateEulerAngles();
  u.MahonyAHRSupdate(msg->xgyro, msg->ygyro, msg->zgyro, msg->xaccl, msg->yaccl, msg->zaccl, msg->xmagn, msg->ymagn, msg->zmagn);
//  ROS_INFO("Quaternions: [%.3f, %.3f, %.3f, %0.3f]", u.getQuaternions(0), u.getQuaternions(1), u.getQuaternions(2), u.getQuaternions(3));
  ROS_INFO("Euler angles: [%.3f, %.3f, %.3f]", u.getEulerAngles(0), u.getEulerAngles(1), u.getEulerAngles(2));
  
  /* Publist rviz information */
  static tf::TransformBroadcaster tfbc;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(-2,-2,-2) );
  tf::Quaternion q(u.getQuaternions(3),u.getQuaternions(0),u.getQuaternions(1),u.getQuaternions(2));
//  q.setRPY(0,0,1);
  transform.setRotation(q);
  tfbc.sendTransform( tf::StampedTransform(transform, ros::Time::now(), "map", "bar"));
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
