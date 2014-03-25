#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "aauship/Faps.h"

#include <sstream> // For stupid CPP handling

// note on plain values:
// buttons are either 0 or 1
// button axes go from 0 to -1
// stick axes go from 0 to +/-1

#define PS3_BUTTON_SELECT            0
#define PS3_BUTTON_STICK_LEFT        1
#define PS3_BUTTON_STICK_RIGHT       2
#define PS3_BUTTON_START             3
#define PS3_BUTTON_CROSS_UP          4
#define PS3_BUTTON_CROSS_RIGHT       5
#define PS3_BUTTON_CROSS_DOWN        6
#define PS3_BUTTON_CROSS_LEFT        7
#define PS3_BUTTON_REAR_LEFT_2       8
#define PS3_BUTTON_REAR_RIGHT_2      9
#define PS3_BUTTON_REAR_LEFT_1       10
#define PS3_BUTTON_REAR_RIGHT_1      11
#define PS3_BUTTON_ACTION_TRIANGLE   12
#define PS3_BUTTON_ACTION_CIRCLE     13
#define PS3_BUTTON_ACTION_CROSS      14
#define PS3_BUTTON_ACTION_SQUARE     15
#define PS3_BUTTON_PAIRING           16

#define PS3_AXIS_STICK_LEFT_LEFTWARDS    0
#define PS3_AXIS_STICK_LEFT_UPWARDS      1
#define PS3_AXIS_STICK_RIGHT_LEFTWARDS   2
#define PS3_AXIS_STICK_RIGHT_UPWARDS     3
#define PS3_AXIS_BUTTON_CROSS_UP         4
#define PS3_AXIS_BUTTON_CROSS_RIGHT      5
#define PS3_AXIS_BUTTON_CROSS_DOWN       6
#define PS3_AXIS_BUTTON_CROSS_LEFT       7
#define PS3_AXIS_BUTTON_REAR_LEFT_2      8
#define PS3_AXIS_BUTTON_REAR_RIGHT_2     9
#define PS3_AXIS_BUTTON_REAR_LEFT_1      10
#define PS3_AXIS_BUTTON_REAR_RIGHT_1     11
#define PS3_AXIS_BUTTON_ACTION_TRIANGLE  12
#define PS3_AXIS_BUTTON_ACTION_CIRCLE    13
#define PS3_AXIS_BUTTON_ACTION_CROSS     14
#define PS3_AXIS_BUTTON_ACTION_SQUARE    15
#define PS3_AXIS_ACCELEROMETER_LEFT      16
#define PS3_AXIS_ACCELEROMETER_FORWARD   17
#define PS3_AXIS_ACCELEROMETER_UP        18
#define PS3_AXIS_GYRO_YAW                19



/**
 * This is the joy tele operation node
 */

class JoyTeleOperation
{
public:
  JoyTeleOperation()
  {
    pub = n.advertise<aauship::Faps>("control_input", 1000);
    sub = n.subscribe("joy", 1000, &JoyTeleOperation::chatterCallback, this);
  }

  void chatterCallback(const sensor_msgs::Joy::ConstPtr& msg)
  {
    aauship::Faps msg2;
    msg2.MsgID = 1;
    msg2.DevID = 2;
    std::stringstream ss;
    ss << "hello world ";
    msg2.Data = ss.str();
    msg2.Time = 3.14159;
    pub.publish(msg2);
    ROS_INFO("I heard: [%f, %f]", msg->axes[12], msg->axes[13]); // REAR_LEFT_2, REAR_RIGHT_2
  }

private:
  ros::NodeHandle n;
  ros::Publisher pub;
  ros::Subscriber sub;

}; // End of class JoyTeleOperation

int main(int argc, char **argv)
{
  ros::init(argc, argv, "teleop_node");
  //Create an object of class SubscribeAndPublish that will take care of everything
  JoyTeleOperation SAPObject;
  ros::spin();

  return 0;
}




