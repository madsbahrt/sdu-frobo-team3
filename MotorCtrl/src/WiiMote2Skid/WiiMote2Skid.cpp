#include "ros/ros.h"
#include "std_msgs/String.h"
#include "MotorCtrl/MotorCtrlMsg.h"
#include <boost/lexical_cast.hpp>
#include <boost/format.hpp>
#include "geometry_msgs/TwistStamped.h"
 //   using boost::bad_lexical_cast;


#include <sstream>

ros::Publisher motorCtrl_pub;

void motorCtrlCallback(const geometry_msgs::TwistStamped::ConstPtr& in_msg)
{
	float linear = in_msg->twist.linear.x;
	float angular = in_msg->twist.angular.z;
	ROS_DEBUG("I heard: Linear[%f] Angular[%f]", linear , angular);
	int left = 10000 * linear - 10000*angular;

	int right = 10000 * linear + 10000*angular;

	MotorCtrl::MotorCtrlMsg out_msg;

	ROS_DEBUG("Transform: Left[%i] Right[%i]", left, right);

	out_msg.l_speed = left;
	out_msg.r_speed = right;
	motorCtrl_pub.publish(out_msg);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "MotorCtrl");
	ros::NodeHandle n;
	motorCtrl_pub = n.advertise<MotorCtrl::MotorCtrlMsg>("MotorCtrl", 20);
	ros::Subscriber sub = n.subscribe("fmHMI/wii_cmd_vel", 20, motorCtrlCallback);
	ros::spin();
	return 0;
}
