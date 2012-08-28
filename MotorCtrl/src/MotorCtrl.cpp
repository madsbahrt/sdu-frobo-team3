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
 //   using boost::bad_lexical_cast;


#include <sstream>
#define NEGATIVE 0b1000000000000000

ros::Publisher ul_pub;

std::string convert(int16_t input){
	if(input > 10000) input = 10000;
	if(input < -10000) input = -10000;
	if(input < 0) {
		input = abs(input);
		input = input|NEGATIVE;
	}
	std::stringstream ss;
	ss << boost::format(
				"%04X")
				% input;
	return ss.str();
}

void motorCtrlCallback(const MotorCtrl::MotorCtrlMsg::ConstPtr& in_msg)
{
	ROS_DEBUG("I heard: Left[%d] Right[%d]", in_msg->l_speed, in_msg->r_speed);
	std_msgs::String out_msg;
	std::string out_string = convert(in_msg->l_speed) + convert(in_msg->r_speed);
	ROS_DEBUG("I send: [%s]", out_string.c_str());
	out_msg.data = out_string;
	ul_pub.publish(out_msg);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "MotorCtrl");
	ros::NodeHandle n;
	ul_pub = n.advertise<std_msgs::String>("ULREG_W05", 20);
	ros::Subscriber sub = n.subscribe("MotorCtrl", 20, motorCtrlCallback);
	ros::spin();
	return 0;
}
