/*************************************************************************************
 # Copyright (c) 2011, Kent Stark Olsen
 # All rights reserved.
 #
 # Redistribution and use in source and binary forms, with or without
 # modification, are permitted provided that the following conditions are met:
 # 1. Redistributions of source code must retain the above copyright
 #    notice, this list of conditions and the following disclaimer.
 # 2. Redistributions in binary form must reproduce the above copyright
 #    notice, this list of conditions and the following disclaimer in the
 #    documentation and/or other materials provided with the distribution.
 # 3. All advertising materials mentioning features or use of this software
 #    must display the following acknowledgement:
 #    This product includes software developed by the University of Southern Denmark.
 # 4. Neither the name of the University of Southern Denmark nor the
 #    names of its contributors may be used to endorse or promote products
 #    derived from this software without specific prior written permission.
 #
 # THIS SOFTWARE IS PROVIDED BY KENT STARK OLSEN ''AS IS'' AND ANY
 # EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 # WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 # DISCLAIMED. IN NO EVENT SHALL KENT STARK OLSEN BE LIABLE FOR ANY
 # DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 # (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 # LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 # ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 # (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 # SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 **************************************************************************************
 # File:    wiimote_to_twist.cpp
 # Author:  Kent Stark Olsen <kent.stark.olsen@gmail.com>
 # Created:     Jun 24, 2011 Kent Stark Olsen
 **************************************************************************************
 # Features:
 #  * Makes twist message of Wiimote accelerometer
 #  * Filters input from Wiimote (Runge Kutta 4th order)
 *************************************************************************************/

#include "ros/ros.h"
//#include "joy/Joy.h"
#include "sensor_msgs/Joy.h"
#include "geometry_msgs/TwistStamped.h"
#include "math.h"
#include "boost/circular_buffer.hpp"
#include "std_msgs/Bool.h"

ros::NodeHandle n;
ros::Subscriber joy_sub;
ros::Publisher override_pub;

bool override;

void joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  std_msgs::Bool msg;
  override_pub.publish(msg);
}

int main(int argc, char **argv)
{
  //  Initialize ROS
  ros::init(argc, argv, "wiiOverride");
  n = ros::NodeHandle();
  override = false;

  joy_sub = n.subscribe<sensor_msgs::Joy> ("joy", 10, joyCallback);
  override_pub = n.advertise<std_msgs::Bool> ("wiiOverride", 1);

  //  Loop
  ros::spin();
}
