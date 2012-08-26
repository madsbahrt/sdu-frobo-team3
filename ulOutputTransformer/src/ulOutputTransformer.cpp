#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "ulOutputTransformer/encPosMsg.h"
#include "ulOutputTransformer/encSpdMsg.h"
#include <boost/lexical_cast.hpp>
#include <boost/format.hpp>
#include <time.h>


#include <sstream>
#define NEGATIVE_BIT 0b1000000000000000

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

#define NEGATIVE_BIT 0b1000000000000000
#define NUMBER_BITS  0b0111111111111111
#define START_BIT    0x00000001


ros::Publisher enc_pos_pub;
ros::Publisher enc_spd_pub;
ros::Publisher start_press_pub;
time_t lastpress;

uint32_t hexstr2uint_32conv(std::string input){

	uint32_t inttempresult;
	uint32_t shiftedresult;
	uint32_t result;

	for(int i = 0; i<8; i++){
		switch ( input.at(i) ) {

		case ASCII_0 :
			inttempresult = 0;
			break;
		case ASCII_1 :
			inttempresult = 1;
			break;
		case ASCII_2 :
			inttempresult = 2;
			break;
		case ASCII_3 :
			inttempresult = 3;
			break;
		case ASCII_4 :
			inttempresult = 4;
			break;
		case ASCII_5 :
			inttempresult = 5;
			break;
		case ASCII_6 :
			inttempresult = 6;
			break;
		case ASCII_7 :
			inttempresult = 7;
			break;
		case ASCII_8 :
			inttempresult = 8;
			break;
		case ASCII_9 :
			inttempresult = 9;
			break;
		case ASCII_A :
		case ASCII_a :
			inttempresult = 10;
			break;
		case ASCII_B :
		case ASCII_b :
			inttempresult = 11;
			break;
		case ASCII_C :
		case ASCII_c :
			inttempresult = 12;
			break;
		case ASCII_D :
		case ASCII_d :
			inttempresult = 13;
			break;
		case ASCII_E :
		case ASCII_e :
			inttempresult = 14;
			break;
		case ASCII_F :
		case ASCII_f :
			inttempresult = 15;
			break;
		default :
			ROS_ERROR("Tried to hexconvert character ", input.at(i));
			break;
		}

		ROS_INFO("Hexconverted [%s] to [%i]", input.substr(i, 1).c_str(), inttempresult);
		shiftedresult = inttempresult << ((7-i)*4);
		result += shiftedresult;
		ROS_INFO("shifted [%X] to [%X], accumulated result now [%X]", inttempresult, shiftedresult, result);
	}
	return result;
}

int16_t uint2int(uint16_t input){
	int16_t result = 0;
	if((input & NEGATIVE_BIT) == 0){ //positive number
		result = input;
    } else { //negative number
    	result = -(input & NUMBER_BITS);
    }
	ROS_INFO("Signconverted [%X] to [%i]", input, result);
	return result;
}

void ulR01Callback(const std_msgs::String::ConstPtr& in_msg)
{
	ulOutputTransformer::encPosMsg encPosMsg;
	ROS_INFO("I heard: [%s]", in_msg->data.c_str());
    uint32_t raw_bits = hexstr2uint_32conv(in_msg->data);

    uint16_t high_bits = raw_bits >> 16;
    uint16_t low_bits = raw_bits;
    ROS_INFO("Highbits [%X] Lowbits [%X]", high_bits, low_bits);

    encPosMsg.l_pos = uint2int(high_bits);
    encPosMsg.r_pos = uint2int(low_bits);
    enc_pos_pub.publish(encPosMsg);
}

void ulR02Callback(const std_msgs::String::ConstPtr& in_msg)
{
	ulOutputTransformer::encSpdMsg encSpdMsg;
	ROS_INFO("I heard: [%s]", in_msg->data.c_str());
    uint32_t raw_bits = hexstr2uint_32conv(in_msg->data);

    uint16_t high_bits = raw_bits >> 16;
    uint16_t low_bits = raw_bits;
    ROS_INFO("Highbits [%X] Lowbits [%X]", high_bits, low_bits);

    encSpdMsg.l_speed = uint2int(high_bits);
    encSpdMsg.r_speed = uint2int(low_bits);
    enc_spd_pub.publish(encSpdMsg);
}
void ulR03Callback(const std_msgs::String::ConstPtr& in_msg)
{
	ROS_INFO("I heard: [%s]", in_msg->data.c_str());
    uint32_t raw_bits = hexstr2uint_32conv(in_msg->data);
    if(((raw_bits  & START_BIT)!=0) && (time (NULL) - lastpress > 20)){ //start
    	std_msgs::Bool startMsg;
    	startMsg.data = true;
    	lastpress = time (NULL);

    	start_press_pub.publish(startMsg);

    } else {
    	ROS_INFO("Startbit false");
    }
    ROS_INFO("Timer: [%i]", time (NULL));
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "ulOutputTransformer");
	ros::NodeHandle n;
	enc_pos_pub = n.advertise<ulOutputTransformer::encPosMsg>("enc_pos", 20);
	enc_spd_pub = n.advertise<ulOutputTransformer::encSpdMsg>("enc_spd", 20);
	start_press_pub = n.advertise<std_msgs::Bool>("start", 20);
	ros::Subscriber r01sub = n.subscribe("ULREG_R01", 20, ulR01Callback);
	ros::Subscriber r02sub = n.subscribe("ULREG_R02", 20, ulR02Callback);
	ros::Subscriber r03sub = n.subscribe("ULREG_R03", 20, ulR03Callback);
	lastpress = time (NULL);
	ros::spin();
	return 0;
}
