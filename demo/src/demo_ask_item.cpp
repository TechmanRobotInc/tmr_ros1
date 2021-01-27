// ROS headers
#include <ros/ros.h>
#include <std_msgs/String.h>

// std header
#include <sstream>
#include <cstdlib>

// TM Driver header
#include "tm_msgs/AskItem.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "demo_ask_item");      
  ros::NodeHandle nh_demo; 
  ros::ServiceClient client = nh_demo.serviceClient<tm_msgs::AskItem>("tm_driver/ask_item");
  tm_msgs::AskItem srv;

  //Request 
  srv.request.id = "demo";
  srv.request.item = "HandCamera_Value";
  srv.request.wait_time = 1;

  // Wait for the result.
  if (client.call(srv))                             
  {
    if (srv.response.ok) {
      ROS_INFO_STREAM("AskItem to robot: id is " << srv.response.id << ", value is " << srv.response.value);
    }    	
    else { 
      ROS_WARN_STREAM("AskItem to robot , but response not yet ok ");
    }    	
  }
  else
  {
    ROS_ERROR_STREAM("Error AskItem to robot");
    return 1;
  }

  //ROS_INFO_STREAM_NAMED("AskItem", "shutdown.");  	
  return 0;
}
