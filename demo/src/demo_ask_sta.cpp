// ROS headers
#include <ros/ros.h>
#include <std_msgs/String.h>

// std header
#include <sstream>
#include <cstdlib>

// TM Driver header
#include "tm_msgs/AskSta.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "demo_ask_sta");      
  ros::NodeHandle nh_demo; 
  ros::ServiceClient client = nh_demo.serviceClient<tm_msgs::AskSta>("tm_driver/ask_sta");
  tm_msgs::AskSta srv;

  //Request  
  srv.request.subcmd = "00";
  srv.request.subdata = "";
  srv.request.wait_time = 1;

  // Wait for the result.
  if (client.call(srv))                             
  {
    if (srv.response.ok) {
    	ROS_INFO_STREAM("AskSta to robot: subcmd is " << srv.response.subcmd << ", subdata is " << srv.response.subdata); 
    }    	
    else { 
    	ROS_WARN_STREAM("AskSta to robot , but response not yet ok ");
    }    	
  }
  else
  {
    ROS_ERROR_STREAM("Error AskSta to robot");
    return 1;
  }

  //ROS_INFO_STREAM_NAMED("AskSta", "shutdown.");  	
  return 0;
}
