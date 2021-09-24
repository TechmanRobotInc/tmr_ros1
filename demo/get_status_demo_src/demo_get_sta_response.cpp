// ROS headers
#include <ros/ros.h>
#include <std_msgs/String.h>

// TM Driver header
#include "tm_msgs/StaResponse.h"

void StaResponseCallback(const tm_msgs::StaResponse::ConstPtr& msg)
{
  ROS_INFO_STREAM("StaResponse: subcmd is = " << msg->subcmd << ", subdata is " << msg->subdata);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "StaResponse");
  ros::NodeHandle nh_demo_get;
  ros::Subscriber sub = nh_demo_get.subscribe("tm_driver/sta_response", 1000, StaResponseCallback);
  ros::spin();
  return 0;  		  	  		
}
