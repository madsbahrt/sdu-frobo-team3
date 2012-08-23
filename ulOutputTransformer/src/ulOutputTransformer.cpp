#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "ulOutputTransformer/encPosMsg.h"
#include "ulOutputTransformer/encSpdMsg.h"
#include <boost/lexical_cast.hpp>
#include <boost/format.hpp>
 //   using boost::bad_lexical_cast;


#include <sstream>
#define NEGATIVE 0b1000000000000000

ros::Publisher enc_pos_pub;
ros::Publisher enc_spd_pub;
ros::Publisher touch_pub;

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

void ulR01Callback(const std_msgs::String::ConstPtr& in_msg)
{
	//ROS_INFO("I heard: Left[%d] Right[%d]", in_msg->l, in_msg->r);
	std_msgs::String out_msg;
	//std::string out_string = convert(in_msg->l) + convert(in_msg->r);
	//ROS_INFO("I send: [%s]", out_string.c_str());
	//out_msg.data = out_string;
	//ul_pub.publish(out_msg);
}

void ulR02Callback(const std_msgs::String::ConstPtr& in_msg)
{
	//ROS_INFO("I heard: Left[%d] Right[%d]", in_msg->l, in_msg->r);
	std_msgs::String out_msg;
	//std::string out_string = convert(in_msg->l) + convert(in_msg->r);
	//ROS_INFO("I send: [%s]", out_string.c_str());
	//out_msg.data = out_string;
	//ul_pub.publish(out_msg);
}
void ulR03Callback(const std_msgs::String::ConstPtr& in_msg)
{
	//ROS_INFO("I heard: Left[%d] Right[%d]", in_msg->l, in_msg->r);
	std_msgs::String out_msg;
	//std::string out_string = convert(in_msg->l) + convert(in_msg->r);
	//ROS_INFO("I send: [%s]", out_string.c_str());
	//out_msg.data = out_string;
	//ul_pub.publish(out_msg);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "ulOutputTransformer");
	ros::NodeHandle n;
	enc_pos_pub = n.advertise<ulOutputTransformer::encPosMsg>("enc_pos", 20);
	enc_spd_pub = n.advertise<ulOutputTransformer::encSpdMsg>("enc_spd", 20);
	touch_pub = n.advertise<std_msgs::Bool>("start_btn", 20);
	ros::Subscriber r01sub = n.subscribe("ULREG_R01", 20, ulR01Callback);
	ros::Subscriber r02sub = n.subscribe("ULREG_R02", 20, ulR02Callback);
	ros::Subscriber r03sub = n.subscribe("ULREG_R03", 20, ulR03Callback);
	ros::spin();
	return 0;
}
