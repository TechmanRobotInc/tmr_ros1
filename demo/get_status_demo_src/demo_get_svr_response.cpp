// ROS headers
#include <ros/ros.h>
#include <std_msgs/String.h>

// TM Driver header
#include "tm_msgs/SvrResponse.h"

void SvrResponseCallback(const tm_msgs::SvrResponse::ConstPtr& msg)
{
  ROS_INFO_STREAM("SvrResponse: id is = " << msg->id << ", mode is " << msg->mode << ", content is " << msg->content << ", error code is " << msg->error_code); 
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "SvrResponse");
  ros::NodeHandle nh_demo_get;
  ros::Subscriber sub = nh_demo_get.subscribe("tm_driver/svr_response", 1000, SvrResponseCallback);
  ros::spin();
  return 0;  		  	  		
}

