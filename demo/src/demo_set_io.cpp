// ROS headers
#include <ros/ros.h>
#include <std_msgs/String.h>

// std header
#include <sstream>
#include <cstdlib>

// TM Driver header
#include "tm_msgs/SetIO.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "demo_set_io");      
  ros::NodeHandle nh_demo; 
  ros::ServiceClient client = nh_demo.serviceClient<tm_msgs::SetIO>("tm_driver/set_io");
  tm_msgs::SetIO srv;

  //Request  	
  srv.request.module = tm_msgs::SetIO::Request::MODULE_CONTROLBOX;
  srv.request.type = tm_msgs::SetIO::Request::TYPE_DIGITAL_OUT;
  srv.request.pin = 0;
  srv.request.state = tm_msgs::SetIO::Request::STATE_ON;

  if (client.call(srv))                             
  {
    if (srv.response.ok) ROS_INFO_STREAM("SetIO to robot");
    else ROS_WARN_STREAM("SetIO to robot , but response not yet ok ");
  }
  else
  {
    ROS_ERROR_STREAM("Error SetIO to robot");
    return 1;
  }

  //ROS_INFO_STREAM_NAMED("SetIO", "shutdown.");  	
  return 0;
}
