#include "tm_ros_driver_windows.hpp"

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
  std::string re = "[sct]"+current_time()+"->"+msg->id+":"+msg->script;
  send_to_ui_list(re);
}
void RosPage::sta_response_callback(const tm_msgs::StaResponse::ConstPtr& msg){
  std::string re = "[sta]"+current_time()+"->subcmd:"+msg->subcmd+",subdata:"+msg->subdata;
  send_to_ui_list(re);
}
void RosPage::svr_response_callback(const tm_msgs::SvrResponse::ConstPtr& msg){
  std::string re = "[svr]"+current_time()+"->"+msg->id+":mode->"+std::to_string(msg->mode)+", content->"+msg->content+", error_code->"+std::to_string(msg->error_code);
  send_to_ui_list(re);
}
void RosPage::send_service(ros::ServiceClient client,tm_msgs::ConnectTM srv){
  if (client.call(srv))
  {
    ROS_INFO("is ok -> %d", srv.response.ok);
  }
  else
  {
    ROS_ERROR("Failed to call service");
    return ;
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

  ros::ServiceClient client = node.serviceClient<tm_msgs::SetIO>("tm_driver/set_io");

  tm_msgs::SetIO srv;
  srv.request.module = srv.request.MODULE_CONTROLBOX;
  srv.request.type = srv.request.TYPE_DIGITAL_OUT;;
  srv.request.pin = 0;
  if(!lastStatus){
    srv.request.state = srv.request.STATE_ON;
    ROS_INFO("set on");
  } else{
    srv.request.state = srv.request.STATE_OFF;
    ROS_INFO("set off");
  }
  lastStatus = !lastStatus;
  if (client.call(srv))
  {
    ROS_INFO("is ok -> %d", srv.response.ok);
  }
  else
  {
    ROS_ERROR("Failed to call service");
    return ;
  }
  ROS_INFO("success send io");
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
