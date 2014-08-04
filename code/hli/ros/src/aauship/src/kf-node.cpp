#include "ros/ros.h"
#include "aauship/ADIS16405.h"
#include <aauship/KalmanFilter.h>


// This node shall read imu measurements and use the attityde from the AHRS filter to
// publius the rest of the system states and publish them
//
// x_n, y_n, phi, theta, psi, u, v, p, q, r


// Used to configure the filter
void attitudeCallback(const aauship::ADIS16405::ConstPtr& msg)
{
  // @TODO Run KF
  // @TODO Publish kf states
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "kf_node");
  ros::NodeHandle attitude;
  ros::Subscriber attitudesub = attitude.subscribe("attitude", 2, attitudeCallback);
  ros::spin();

  return 0;
}
