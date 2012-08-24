#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "ulOutputTransformer/encPosMsg.h"
#include "ulOutputTransformer/encSpdMsg.h"
#include <boost/lexical_cast.hpp>
#include <boost/format.hpp>

#include <sstream>
#define NEGATIVE 0b1000000000000000

#define ASCII_0 48
#define ASCII_1 49
#define ASCII_2 50
#define ASCII_3 51
#define ASCII_4 52
#define ASCII_5 53
#define ASCII_6 54
#define ASCII_7 55
#define ASCII_8 56
#define ASCII_9 57
#define ASCII_A 65
#define ASCII_a 97
#define ASCII_B 66
#define ASCII_b 98
#define ASCII_C 67
#define ASCII_c 99
#define ASCII_D 68
#define ASCII_d 100
#define ASCII_E 69
#define ASCII_e 101
#define ASCII_F 70
#define ASCII_f 102

ros::Publisher enc_pos_pub;
ros::Publisher enc_spd_pub;
ros::Publisher touch_pub;

int16_t convert(std::string input){

	int results[8];

	uint32_t result;

	bool negative = false;

	for(int i = 0; i<8; i++){
		switch ( input.at(i) ) {

		case ASCII_0 :
			results[i] = 0;
			break;
		case ASCII_1 :
			results[i] = 1;
			break;
		case ASCII_2 :
			results[i] = 2;
			break;
		case ASCII_3 :
			results[i] = 3;
			break;
		case ASCII_4 :
			results[i] = 4;
			break;
		case ASCII_5 :
			results[i] = 5;
			break;
		case ASCII_6 :
			results[i] = 6;
			break;
		case ASCII_7 :
			results[i] = 7;
			break;
		case ASCII_8 :
			results[i] = 8;
			break;
		case ASCII_9 :
			results[i] = 9;
			break;
		case ASCII_A :
		case ASCII_a :
			results[i] = 10;
			break;
		case ASCII_B :
		case ASCII_b :
			results[i] = 11;
			break;
		case ASCII_C :
		case ASCII_c :
			results[i] = 12;
			break;
		case ASCII_D :
		case ASCII_d :
			results[i] = 13;
			break;
		case ASCII_E :
		case ASCII_e :
			results[i] = 14;
			break;
		case ASCII_F :
		case ASCII_f :
			results[i] = 15;
			break;
		default :
			ROS_ERROR("Tried to hexconvert character ", input.at(i));
			break;
		}

		if(i == 0 && result[i] > 7){

		}


		ROS_INFO("Hexconverted [%s] to [%i]", input.substr(i, 1).c_str(), results[i]);
	}
	return 0;//ss.str();
}

void ulR01Callback(const std_msgs::String::ConstPtr& in_msg)
{
	ROS_INFO("I heard: [%s]", in_msg->data.c_str());
    convert(in_msg->data);

//	ROS_INFO("Converted to: [%d]", x);


//	using boost::lexical_cast;
//	using boost::bad_lexical_cast;

//	std::stringstream ss;
//	ss << "0x" << in_msg->data;
//
//	try {
//	    uint32_t x = lexical_cast<uint32_t>(ss.str());
//	    ROS_INFO("Converted to: [%d]", x);
//	} catch(bad_lexical_cast &) {
//	    ROS_ERROR("Failed to convert string %s", ss.str().c_str());
//	}


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
