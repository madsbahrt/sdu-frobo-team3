#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>
#include "fmMsgs/serial.h"
#include <boost/thread.hpp>
#include "ulregs.h"

#define callback(I)  ((X) < (Y) ? (X) : (Y))

ros::Publisher tx_pub;
fmMsgs::serial serial_tx_msg;
bool data_received;
std::string received_data = "00000000";

void w04cb(const std_msgs::String::ConstPtr& msg)
{
	ulregs[4].data = msg->data;
}

void w05cb(const std_msgs::String::ConstPtr& msg)
{
	ulregs[5].data = msg->data;
}

void w06cb(const std_msgs::String::ConstPtr& msg)
{
	ulregs[6].data = msg->data;
}

void w07cb(const std_msgs::String::ConstPtr& msg)
{
	ulregs[7].data = msg->data;
}

int initializeULRegs(ros::NodeHandle n){
	int i;
	for(i = 0; i<8; i++){
		ulregs[i].id = 1;
		if(i<4){
			ulregs[i].direction = IN;
			std::stringstream ss;
			ss << "ULREG_R0" << i;
			ulregs[i].publisher = n.advertise<std_msgs::String>(ss.str(), 20);
			std::stringstream ss2;
			ss2 << "#R:0" << i << "\n";
			ulregs[i].command = ss2.str();
		} else {
			ulregs[i].direction = OUT;
			std::stringstream ss;
			ss << "ULREG_W0" << i;
			switch ( i ) {
			case 4 :
				ulregs[i].subscriber = n.subscribe(ss.str(), 20, w04cb);
				break;
			case 5 :
				ulregs[i].subscriber = n.subscribe(ss.str(), 20, w05cb);
				break;
			case 6 :
				ulregs[i].subscriber = n.subscribe(ss.str(), 20, w06cb);
				break;
			case 7 :
				ulregs[i].subscriber = n.subscribe(ss.str(), 20, w07cb);
				break;
			default :
				ROS_ERROR("This should really not happen...");
			}
			ulregs[i].data = "00000000";
		}
	}
	return 0;
}

void serialCallback(const fmMsgs::serial::ConstPtr& msg)
{
	ROS_INFO("I heard: [%s]", msg->data.c_str());
	data_received = true;
	received_data = msg->data.c_str();
}


int sendMsg(std::string s){
	++serial_tx_msg.header.seq;
	std::stringstream ss;
	ss << s << "\n";
	serial_tx_msg.data = ss.str();
	serial_tx_msg.header.stamp = ros::Time::now();

	ROS_INFO("%s", serial_tx_msg.data.c_str());

	tx_pub.publish(serial_tx_msg);
	return 0;
}

int main(int argc, char **argv){
	ros::init(argc, argv, "unitylink");
	ros::NodeHandle n;
	initializeULRegs(n);
	tx_pub = n.advertise<fmMsgs::serial>("S0_tx_msg", 20);
	ros::Rate loop_rate(100);
	serial_tx_msg.header.seq = 0;
	ros::Subscriber serialsub = n.subscribe("S0_rx_msg", 20, serialCallback);
	data_received = true;
	int cur_reg = 0;
	std_msgs::String string_msg;
	bool first_msg = true;
	int data_not_rcvd_cntr = 0;


	while (ros::ok())
	{
		if(data_received){
			//Handling old register
			data_received = false;
			data_not_rcvd_cntr = 0;
			if(ulregs[cur_reg].direction == IN){
				if(first_msg){
					first_msg = false;
				} else {
					if(received_data.find("#S_R")==0){//Sync
						string_msg.data = received_data = received_data.substr(5, 8);
						ROS_INFO("DATA_RECEIVED R0%d: %s", cur_reg, received_data.c_str());
						ulregs[cur_reg].publisher.publish(string_msg);
					}
				}
			} else {
				ROS_DEBUG("Received reply on W0%d", cur_reg);
			}
			cur_reg++;
			if(cur_reg > 7) cur_reg = 0;
			//Handling next register
			if(ulregs[cur_reg].direction == IN){
				sendMsg(ulregs[cur_reg].command);
			} else {
				std::stringstream ss;
				ss << "#W:0" << cur_reg << " " << ulregs[cur_reg].data << "\n";
				sendMsg(ss.str());
			}
		} else {
			if(++data_not_rcvd_cntr > 100){
				data_not_rcvd_cntr = 0;
				first_msg = true;
				data_received = true;
			}
		}


		ros::spinOnce();

		loop_rate.sleep();
	}


	return 0;
}


