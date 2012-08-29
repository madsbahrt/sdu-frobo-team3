#include "ros/ros.h"
//#include "joy/Joy.h"
#include "sensor_msgs/Joy.h"
#include "geometry_msgs/TwistStamped.h"
#include "math.h"
#include "boost/circular_buffer.hpp"
#include "std_msgs/Bool.h"

ros::Subscriber joy_sub;
ros::Publisher override_pub;

bool override;
time_t lastpress;

void joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  std_msgs::Bool msg;
  if(joy->buttons[2] && (time (NULL) - lastpress > 1)){
	  lastpress = time (NULL);
	  override = !override;
	  msg.data = override;
	  override_pub.publish(msg);
  }
}

int main(int argc, char **argv)
{
  //  Initialize ROS
  ros::init(argc, argv, "wiiOverride");
  ros::NodeHandle n = ros::NodeHandle();
  override = false;

  joy_sub = n.subscribe<sensor_msgs::Joy> ("/fmHMI/joy", 10, joyCallback);
  override_pub = n.advertise<std_msgs::Bool> ("wiiOverride", 1);
  lastpress = time (NULL);
  //  Loop
  ros::spin();
}
