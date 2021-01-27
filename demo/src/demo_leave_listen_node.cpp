// ROS headers
#include <ros/ros.h>
#include <std_msgs/String.h>

// std header
#include <sstream>
#include <cstdlib>

// TM Driver header
#include "tm_msgs/SendScript.h"

int main(int argc, char **argv)
{    
  std::string cmd = "ScriptExit()";

  ros::init(argc, argv, "demo_leave_listen_node");      
  ros::NodeHandle nh_demo; 
  ros::ServiceClient client = nh_demo.serviceClient<tm_msgs::SendScript>("tm_driver/send_script");
  tm_msgs::SendScript srv;
  
  //Request	
  srv.request.id = "demo";
  srv.request.script = cmd;

  if (client.call(srv))                             
  {
    if (srv.response.ok) ROS_INFO_STREAM("Exit script to robot");
    else ROS_WARN_STREAM("Exit script to robot , but response not yet ok ");
  }
  else
  {
    ROS_ERROR_STREAM("Error Exit script to robot");
    return 1;
  }    	

  ROS_INFO_STREAM_NAMED("ScriptExit", "shutdown.");  	
  return 0;
}
