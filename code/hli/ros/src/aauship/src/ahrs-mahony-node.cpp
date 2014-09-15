#include "ros/ros.h"
#include "aauship/ADIS16405.h"
#include "geometry_msgs/Quaternion.h"
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <aauship/MahonyAHRS.h>
#include <aauship/MadgwickAHRS.h>

// This node shall read imu measurements and use an AHRS filter to
// calculate attitude and publish such that it can render something
// in rviz.

// Construct filter
MahonyAHRS u(18, 8, 22); // Magic numbers here


class SubscribeAndPublish
{

public:
  SubscribeAndPublish()
  {
    adissub = n.subscribe("imu", 1, &SubscribeAndPublish::adisCallback, this);
    ahrssub = n.subscribe("ahrscfg", 2, &SubscribeAndPublish::ahrscfgCallback, this); // not implemented, supposed to be live update of filter parameters
    attitudepub = n.advertise<geometry_msgs::Quaternion>("attitude", 2, true);
  }

  void adisCallback(const aauship::ADIS16405::ConstPtr& msg)
  {
    /* AHRS update */
    u.MahonyAHRSupdate(
      (msg->xgyro-0.2888)*3.1415/180, (msg->ygyro-0.1282)*3.1415/180, (msg->zgyro-0.3322)*3.1415/180,
      -msg->xaccl, -msg->yaccl, -msg->zaccl,
      msg->xmagn, msg->ymagn, msg->zmagn);

    /* Mahony filter results */
    tf::Quaternion q(u.getQuaternions(1),u.getQuaternions(2),u.getQuaternions(3),u.getQuaternions(0));
    //tf::Quaternion q(u.getQuaternions(0),u.getQuaternions(1),u.getQuaternions(2),u.getQuaternions(3));

    /* Debug output */
    //u.calculateEulerAngles();
    //ROS_INFO("Euler angles: [%.3f, %.3f, %.3f]", u.getEulerAngles(0), u.getEulerAngles(1), u.getEulerAngles(2));

    /* Publish rviz transform information */
    static tf::TransformBroadcaster tfbc;
    tf::Transform transform;
    transform.setOrigin( tf::Vector3(0,0,0) );
    transform.setRotation(q);
    //tfbc.sendTransform( tf::StampedTransform(transform, ros::Time::now(), "map", "boat_link"));

    // Publish attitude information (for use with i.e. the Kalman filter)
    geometry_msgs::Quaternion msgq;
    msgq.x = u.getQuaternions(1);
    msgq.y = u.getQuaternions(2);
    msgq.z = u.getQuaternions(3);
    msgq.w = u.getQuaternions(0);
    attitudepub.publish(msgq);

  }

  // Used to configure the filter
  void ahrscfgCallback(const aauship::ADIS16405::ConstPtr& msg)
  {
  //  MahonyAHRS::setTuning(float kp, float ki)
  }

private:
  ros::NodeHandle n;
  ros::Subscriber adissub;
  ros::Subscriber ahrssub;
  ros::Publisher attitudepub;

}; //End of class SubscribeAndPublish

// Main
int main(int argc, char **argv)
{
  ros::init(argc, argv, "ahrs_mahony_node");

  //Create an object of class SubscribeAndPublish that will take care of everything
  SubscribeAndPublish SAPObject;

  ros::spin();

  return 0;
}
