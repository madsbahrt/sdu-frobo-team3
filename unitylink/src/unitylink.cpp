#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>
#include "fmMsgs/serial.h"
#include <boost/thread.hpp>

ros::Publisher tx_pub;
fmMsgs::serial serial_tx_msg;


void serialCallback(const fmMsgs::serial::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
  //gem data
  //unlock
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
  ros::init(argc, argv, "talker");
  ros::NodeHandle n;
  tx_pub = n.advertise<fmMsgs::serial>("S0_tx_msg", 20);
  ros::Rate loop_rate(10);
  serial_tx_msg.header.seq = 0;
  ros::Subscriber serialsub = n.subscribe("S0_rx_msg", 20, serialCallback);
  

  while (ros::ok())
  {
    sendMsg("#R:00");
    //lock();

    //skriv resultat til nyt rostopic
    //send næste fors

    ros::spinOnce();

    loop_rate.sleep();
  }


  return 0;
}


