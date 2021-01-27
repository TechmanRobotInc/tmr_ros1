// ROS headers
#include <ros/ros.h>
#include "std_msgs/String.h"

// std header
#include <sstream>
#include <cstdlib>

// TM Driver header
#include "tm_msgs/WriteItem.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "demo_write_item");      
  ros::NodeHandle nh_demo; 
  ros::ServiceClient client = nh_demo.serviceClient<tm_msgs::WriteItem>("tm_driver/write_item");
  tm_msgs::WriteItem srv;

  //Request  	
  srv.request.id = "123";
  srv.request.item = "Ctrl_DO0";
  srv.request.value = "1";

  // Check 
  if (client.call(srv))                             
  {
    if (srv.response.ok) ROS_INFO_STREAM("WriteItem to robot");
    else ROS_WARN_STREAM("WriteItem to robot , but response not yet ok ");
  }
  else
  {
    ROS_ERROR_STREAM("Error WriteItem to robot");
    return 1;
  }

  //ROS_INFO_STREAM_NAMED("WriteItem", "shutdown.");  	
  return 0;
}
