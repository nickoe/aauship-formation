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
MahonyAHRS u(8, 0.4, 12);
MadgwickAHRS p(1,12);

void adisCallback(const aauship::ADIS16405::ConstPtr& msg)
{
  /* AHRS update */
//Gyroscope = [Gyroscope(:,1)+4.6 Gyroscope(:,2)+16.15 Gyroscope(:,3)+5.4];
  u.MahonyAHRSupdate((msg->xgyro)*3.1415/180, (msg->ygyro)*3.1415/180, (msg->zgyro)*3.1415/180,
		    (-msg->xaccl), (-msg->yaccl), (-msg->zaccl),
//		     msg->xmagn*0.0005-0.28, msg->ymagn*0.0005-0.15, msg->zmagn*0.0005-(-0.18));
		     msg->xmagn, msg->ymagn, msg->zmagn);

  p.MadgwickAHRSupdate((msg->xgyro)*3.1415/180, (msg->ygyro)*3.1415/180, (msg->zgyro)*3.1415/180,
		    (-msg->xaccl), (-msg->yaccl), (-msg->zaccl),
//		     msg->xmagn*0.0005-0.28, msg->ymagn*0.0005-0.15, msg->zmagn*0.0005-(-0.18));
		     msg->xmagn, msg->ymagn, msg->zmagn);

//0.28 0.15 -0.18
  printf("gyro [% f, % f, % f]\r\n", (msg->xgyro-1.65)*3.1415/180, (msg->ygyro+16.15)*3.1415/180, (msg->zgyro+5.4)*3.1415/180);
  printf("accl [% f, % f, % f]\r\n", (msg->xaccl), (msg->yaccl), (msg->zaccl));
  printf("magn [% f, % f, % f]\r\n", msg->xmagn, msg->ymagn, msg->zmagn);
  
  /* AHRS debug output */
//  ROS_INFO("Quaternions: [%.3f, %.3f, %.3f, %0.3f]", u.getQuaternions(1), u.getQuaternions(2), u.getQuaternions(3), u.getQuaternions(0));
  u.calculateEulerAngles();
  
  /* Publish rviz transform information */
  static tf::TransformBroadcaster tfbc;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(0,0,0) );

	/* Mahony filter results */
  tf::Quaternion q(u.getQuaternions(1),u.getQuaternions(2),u.getQuaternions(3),u.getQuaternions(0));
  /* Madgwick filter results */
//  tf::Quaternion q(p.getQuaternions(1),p.getQuaternions(2),p.getQuaternions(3),p.getQuaternions(0));

  transform.setRotation(q);
  tfbc.sendTransform( tf::StampedTransform(transform, ros::Time::now(), "map", "boat_link"));
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
  ros::Subscriber adissub = adis.subscribe("imu", 1, adisCallback);
  ros::NodeHandle ahrs;
  ros::Subscriber ahrssub = ahrs.subscribe("ahrs", 2, ahrsCallback);
  ros::spin();

  return 0;
}
