/*
Copyright (C) <year> <copyright holders>

Permission is hereby granted, free of charge,
to any person obtaining a copy of this software
and associated documentation files (the "Software"),
to deal in the Software without restriction,
including without limitation the rights to use,
copy, modify, merge, publish, distribute,
sublicense, and/or sell copies of the Software, and
to permit persons to whom the Software is
furnished to do so, subject to the following
conditions:

The above copyright notice and this permission notice
shall be included in all copies or substantial portions
of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF
ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT
LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO
EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE
FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN
AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
OTHER DEALINGS IN THE SOFTWARE.
 */
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "MotorCtrl/MotorCtrlMsg.h"
#include <boost/lexical_cast.hpp>
#include <boost/format.hpp>
#include "geometry_msgs/TwistStamped.h"
 //   using boost::bad_lexical_cast;


#include <sstream>
#define HALF_WIDTH 0.2
#define X HALF_WIDTH
#define R0 0.4
#define F0 ((R0+X)/(R0-X))
#define SLOWDOWN_FACTOR ((F0+1)/(2*F0))


float f(float angular){
	return (abs(angular)*(F0-1))+1;
}

ros::Publisher motorCtrl_pub;

void motorCtrlCallback(const geometry_msgs::TwistStamped::ConstPtr& in_msg)
{
	float linear = in_msg->twist.linear.x;
	float angular = in_msg->twist.angular.z;
	ROS_DEBUG("I heard: Linear[%f] Angular[%f]", linear , angular);

	int right;
	int left;

	float f_cur = f(angular);

	if(angular < 0){
		right = linear * ((2*f_cur)/(f_cur+1)) * SLOWDOWN_FACTOR;
		left = linear * ((2*f_cur)/(f_cur+1)) * SLOWDOWN_FACTOR;
	} else {
		right;
		left;
	}
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
