// ROS headers
#include <ros/ros.h>
#include <std_msgs/String.h>

// std header
#include <sstream>
#include <cstdlib>

// TM Driver header
#include "tm_msgs/ConnectTM.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "demo_connect_tm");      
  ros::NodeHandle nh_demo; 
  ros::ServiceClient client = nh_demo.serviceClient<tm_msgs::ConnectTM>("tm_diver/connect_tm");
  tm_msgs::ConnectTM srv;

  //Request
  srv.request.server = srv.request.TMSVR;
  srv.request.reconnect = true;
  srv.request.timeout = 0;
  srv.request.timeval = 0;

  if (client.call(srv))                             
  {
    if (srv.response.ok) ROS_INFO_STREAM("ConnectTM to robot");
    else ROS_WARN_STREAM("ConnectTM to robot , but response not yet ok ");
  }
  else
  {
    ROS_ERROR_STREAM("Error ConnectTM to robot");
    return 1;
  }

  //ROS_INFO_STREAM_NAMED("ConnectTM", "shutdown.");  	
  return 0;
}
