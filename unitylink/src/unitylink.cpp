#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>
#include "fmMsgs/serial.h"

int main(int argc, char **argv){
  ros::init(argc, argv, "talker");
  ros::NodeHandle n;
  ros::Publisher tx_pub = n.advertise<fmMsgs::serial>("S0_tx_msg", 20);
  ros::Rate loop_rate(10);
  fmMsgs::serial serial_tx_msg;
  serial_tx_msg.header.seq = 0;

  while (ros::ok())
  {
    ++serial_tx_msg.header.seq;
    serial_tx_msg.data = "#R:00\n";
    serial_tx_msg.header.stamp = ros::Time::now();
    //std::stringstream ss;
    //ss << "hello world " << count;
    //msg.data = ss.str();

    ROS_INFO("%s", serial_tx_msg.data.c_str());

    tx_pub.publish(serial_tx_msg);

    ros::spinOnce();

    loop_rate.sleep();
  }


  return 0;
}


