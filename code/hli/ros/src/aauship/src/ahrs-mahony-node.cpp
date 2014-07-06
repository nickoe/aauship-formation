#include "ros/ros.h"
#include "aauship/ADIS16405.h"
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <aauship/test.h>

// This node shall read imu measurements and use an AHRS filter to
// calculate attitude and publish such that it can render something
// in rviz.

// Construct filter
//AHRS u(8.8, 0.1, 10);
AHRS u(10, 0.4, 11);

void adisCallback(const aauship::ADIS16405::ConstPtr& msg)
{
  /* AHRS update */
//Gyroscope = [Gyroscope(:,1)+4.6 Gyroscope(:,2)+16.15 Gyroscope(:,3)+5.4];
  u.MahonyAHRSupdate((msg->xgyro*0.05-1.65)*3.1415/180, (msg->ygyro*0.05+16.15)*3.1415/180, (msg->zgyro*0.05+5.4)*3.1415/180,
//    u.MahonyAHRSupdate(0,0,0,
		    (msg->xaccl*0.00333), (msg->yaccl*0.00333), (-msg->zaccl*0.00333),
		     msg->xmagn*0.0005, msg->ymagn*0.0005, msg->zmagn*0.0005);
  printf("accl [% f, % f, % f]\r\n", (msg->xaccl*0.00333), (msg->yaccl*0.00333), (msg->zaccl*0.00333));
  printf("gyro [% f, % f, % f]\r\n", msg->xgyro*0.05*3.1415/180, msg->ygyro*0.05*3.1415/180, msg->zgyro*0.05*3.1415/180);
  printf("magn [% f, % f, % f]\r\n", msg->xmagn*0.0005, msg->ymagn*0.0005, msg->zmagn*0.0005);
  
  /* AHRS debug output */
//  ROS_INFO("Quaternions: [%.3f, %.3f, %.3f, %0.3f]", u.getQuaternions(1), u.getQuaternions(2), u.getQuaternions(3), u.getQuaternions(0));
  u.calculateEulerAngles();
//  ROS_INFO("Euler angles: [%.3f, %.3f, %.3f]", u.getEulerAngles(0), u.getEulerAngles(1), u.getEulerAngles(2));
//  ROS_INFO("getTuning: [%.3f, %.3f]", u.getTuning(0), u.getTuning(1));
//  ROS_INFO("sampleFreq: [%.3f]", u.getSampleFreq());
  
  /* Publish rviz information */
  static tf::TransformBroadcaster tfbc;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(0,0,0) );
  tf::Quaternion q(u.getQuaternions(1),u.getQuaternions(2),u.getQuaternions(3),u.getQuaternions(0));
//  tf::Quaternion q;
//  q.setRPY(u.getEulerAngles(0), u.getEulerAngles(1), u.getEulerAngles(2));
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
  ros::Subscriber adissub = adis.subscribe("adis16405", 2, adisCallback);
//  ros::Subscriber adissub = adis.subscribe("imu", 1, adisCallback);
  ros::NodeHandle ahrs;
  ros::Subscriber ahrssub = ahrs.subscribe("ahrs", 2, ahrsCallback);
  ros::spin();

  return 0;
}
