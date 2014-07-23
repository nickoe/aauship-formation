#include "ros/ros.h"
#include "aauship/ADIS16405.h"
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <aauship/MahonyAHRS.h>
#include <aauship/MadgwickAHRS.h>

// This node shall read imu measurements and use an AHRS filter to
// calculate attitude and publish such that it can render something
// in rviz.

// Construct filter
MahonyAHRS u(18, 0.1, 22);

void adisCallback(const aauship::ADIS16405::ConstPtr& msg)
{
  /* AHRS update */
  u.MahonyAHRSupdate((msg->xgyro-0.2888)*3.1415/180, (msg->ygyro-0.1282)*3.1415/180, (msg->zgyro-0.3322)*3.1415/180,
		    (-msg->xaccl), (-msg->yaccl), (-msg->zaccl),
//		     msg->xmagn*0.0005-0.28, msg->ymagn*0.0005-0.15, msg->zmagn*0.0005-(-0.18));
		     msg->xmagn, msg->ymagn, msg->zmagn);

  printf("gyro [% f, % f, % f]\r\n", (msg->xgyro-0.2888)*3.1415/180, (msg->ygyro-0.1282)*3.1415/180, (msg->zgyro-0.3322)*3.1415/180);
  printf("accl [% f, % f, % f]\r\n", (msg->xaccl), (msg->yaccl), (msg->zaccl));
  printf("magn [% f, % f, % f]\r\n", msg->xmagn, msg->ymagn, msg->zmagn);
  
  /* Mahony filter results */
  tf::Quaternion q(u.getQuaternions(1),u.getQuaternions(2),u.getQuaternions(3),u.getQuaternions(0));

  /* Publish rviz transform information */
  static tf::TransformBroadcaster tfbc;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(0,0,0) );
  transform.setRotation(q);
  tfbc.sendTransform( tf::StampedTransform(transform, ros::Time::now(), "map", "boat_link"));

  // @TODO Publish attitude information to the Kalman filter
}

// Used to configure the filter
void ahrsCallback(const aauship::ADIS16405::ConstPtr& msg)
{
//  setTuning(float kpA, float kiA, float kpM, float kiM);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ahrs_mahony_node");
  ros::NodeHandle adis;
  ros::Subscriber adissub = adis.subscribe("imu", 1, adisCallback);
  ros::NodeHandle ahrs;
  ros::Subscriber ahrssub = ahrs.subscribe("ahrs", 2, ahrsCallback);
  ros::spin();

  return 0;
}
