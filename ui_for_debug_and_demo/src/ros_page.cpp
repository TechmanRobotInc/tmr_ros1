/*****************************************************************************
** Includes
*****************************************************************************/
#include "tm_ros_driver_windows.hpp"

using namespace std::chrono_literals;

/*****************************************************************************
** Implementation
*****************************************************************************/
void RosPage::feedback_states_callback(const tm_msgs::FeedbackState::ConstPtr& msg){
  send_ui_feed_back_status(*msg);
}
std::string RosPage::current_time() {
  system_clock::time_point tp = system_clock::now();
 
  time_t raw_time = system_clock::to_time_t(tp);
 
  struct tm  *timeinfo  = std::localtime(&raw_time);
 
  char buf[24] = {0};
  strftime(buf, 24, "%Y-%m-%d %H:%M:%S:", timeinfo);

  std::chrono::milliseconds ms = std::chrono::duration_cast<std::chrono::milliseconds>(tp.time_since_epoch());
 
  std::string milliseconds_str =  std::to_string(ms.count() % 1000);
 
  if (milliseconds_str.length() < 3) {
      milliseconds_str = std::string(3 - milliseconds_str.length(), '0') + milliseconds_str;
  }
 
  return std::string(buf) + milliseconds_str;
}
void RosPage::sct_response_callback(const tm_msgs::SctResponse::ConstPtr& msg){
  std::string re = current_time()+" [sct]-> id:"+msg->id+", script:"+msg->script;
  send_to_ui_list(re);
}
void RosPage::sta_response_callback(const tm_msgs::StaResponse::ConstPtr& msg){
  std::string re = current_time()+" [sta]-> subcmd:"+msg->subcmd+", subdata:"+msg->subdata;
  send_to_ui_list(re);
}
void RosPage::svr_response_callback(const tm_msgs::SvrResponse::ConstPtr& msg){
  std::string re = current_time()+" [svr]-> id:"+msg->id+", mode:"+std::to_string(msg->mode)+", content->"+msg->content+", error_code->"+std::to_string(msg->error_code);
  send_to_ui_list(re);
}
void RosPage::initial_subscriber(){
  feedBackStatusSubscription = node.subscribe("feedback_states", 1000, &RosPage::feedback_states_callback, this);
  sctResponseSubscription = node.subscribe("tm_driver/sct_response", 1000, &RosPage::sct_response_callback, this);
  staResponseSubscription = node.subscribe("tm_driver/sta_response", 1000, &RosPage::sta_response_callback, this);
  svrResponseSubscription = node.subscribe("tm_driver/svr_response", 1000, &RosPage::svr_response_callback, this);
}
void RosPage::initial_client(){
  connectClient = node.serviceClient<tm_msgs::ConnectTM>("tm_diver/connect_tm");
}
void RosPage::send_service(ros::ServiceClient client,tm_msgs::ConnectTM srv){
  if (client.call(srv))
  {
    ROS_INFO_STREAM("srv.response.ok is " << srv.response.ok);
  }
  else
  {
    ROS_ERROR_STREAM("Failed to call service");
    return;
  }
}
void RosPage::send_sct_as_re_connect(){
  tm_msgs::ConnectTM srv;
  srv.request.server = srv.request.TMSCT;
  srv.request.reconnect = true;
  srv.request.timeout = 0;
  srv.request.timeval = 0;
  send_service(connectClient, srv);
}
void RosPage::send_svr_as_re_connect(){
  tm_msgs::ConnectTM srv;
  srv.request.server = srv.request.TMSVR;
  srv.request.reconnect = true;
  srv.request.timeout = 0;
  srv.request.timeval = 0;
  
  send_service(connectClient, srv);
}
void RosPage::change_control_box_io_button(){
  //std::cout<<"ready to send io"<<std::endl;
  ros::ServiceClient client = node.serviceClient<tm_msgs::SetIO>("tm_driver/set_io");
  
  tm_msgs::SetIO srv;
  srv.request.module = srv.request.MODULE_CONTROLBOX;
  srv.request.type = srv.request.TYPE_DIGITAL_OUT;
  srv.request.pin = 0;
  if(!lastStatus){
    srv.request.state = srv.request.STATE_ON;
    ROS_INFO_STREAM("set on");
  } else{
    srv.request.state = srv.request.STATE_OFF;
    ROS_INFO_STREAM("set off");
  }
  lastStatus = !lastStatus;
  
  if (client.call(srv))
  {
    if (srv.response.ok) ROS_INFO_STREAM("SetIO to robot");
    else ROS_WARN_STREAM("SetIO to robot , but response not yet ok ");
  }
  else
  {
    ROS_ERROR_STREAM("Error SetIO to robot");
    return;
  }
  ROS_INFO_STREAM("Send IO success");

}
RosPage::RosPage()
 :lastStatus(false)
 {
  initial_subscriber();
  initial_client();
}
RosPage::~RosPage(){
  
  std::cout<<"call ~RosPage"<<std::endl;
}
void RosPage::run(){
  ros::spin();
}
