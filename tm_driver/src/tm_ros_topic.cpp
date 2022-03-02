#include "tm_driver/tm_ros_node.h"

////////////////////////////////
// Topic
////////////////////////////////

void TmRosNode::publish_fbs()
{
    PubMsg &pm = pm_;
    TmRobotState &state = iface_.state;

    // Publish feedback state
    pm.fbs_msg.header.stamp = ros::Time::now();
    if(state.get_receive_state() != TmCommRC::TIMEOUT){
      pm.fbs_msg.is_svr_connected = iface_.svr.is_connected();
      pm.fbs_msg.is_sct_connected = iface_.sct.is_connected() & iface_.is_on_listen_node();
      pm.fbs_msg.tmsrv_cperr = (int)iface_.svr.tmSvrErrData.error_code();  //Node State Response 
      pm.fbs_msg.tmsrv_dataerr = (int)pm.svr_msg.error_code;
      pm.fbs_msg.tmscript_cperr = (int)iface_.sct.tmSctErrData.error_code();
      pm.fbs_msg.tmscript_dataerr = (int)iface_.sct.sct_data.sct_has_error();
    } else{
      pm.fbs_msg.is_svr_connected = false;
      pm.fbs_msg.is_sct_connected = false;
      pm.fbs_msg.tmsrv_cperr = false;
      pm.fbs_msg.tmsrv_dataerr = false;
      pm.fbs_msg.tmscript_cperr = false;
      pm.fbs_msg.tmscript_dataerr = false;  
      iface_.svr.tmSvrErrData.set_CPError(TmCPError::Code::Ok);
      pm.svr_msg.error_code = false;
      iface_.sct.tmSctErrData.set_CPError(TmCPError::Code::Ok);
      iface_.sct.sct_data.set_sct_data_has_error(false);
    }    
   
    pm.fbs_msg.max_not_connect_in_s = maxNotConnectTimeInS;
    pm.fbs_msg.disconnection_times = diconnectTimes;

    pm.fbs_msg.is_data_table_correct = state.is_data_table_correct();
    pm.fbs_msg.joint_pos = state.joint_angle();
    pm.fbs_msg.joint_vel = state.joint_speed();
    pm.fbs_msg.joint_tor = state.joint_torque();
    pm.fbs_msg.tool_pose = state.tool_pose();
    pm.fbs_msg.tcp_speed = state.tcp_speed_vec();
    pm.fbs_msg.tcp_force = state.tcp_force_vec();
    pm.fbs_msg.robot_link = state.is_linked();
    pm.fbs_msg.robot_error = state.has_error();
    pm.fbs_msg.project_run = state.is_project_running();
    pm.fbs_msg.project_pause = state.is_project_paused();
    pm.fbs_msg.safetyguard_a = state.is_safeguard_A();
    pm.fbs_msg.e_stop = state.is_EStop();
    pm.fbs_msg.camera_light = state.camera_light();
    pm.fbs_msg.error_code = state.error_code();
    pm.fbs_msg.project_speed = state.project_speed();
    pm.fbs_msg.ma_mode = state.ma_mode();
    pm.fbs_msg.robot_light = state.robot_light();
    pm.fbs_msg.cb_digital_output = state.ctrller_DO();
    pm.fbs_msg.cb_digital_input = state.ctrller_DI();
    pm.fbs_msg.cb_analog_output = state.ctrller_AO();
    pm.fbs_msg.cb_analog_input = state.ctrller_AI();
    pm.fbs_msg.ee_digital_output = state.ee_DO();
    pm.fbs_msg.ee_digital_input = state.ee_DI();
    //pm.fbs_msg.ee_analog_output = state.ee_AO();
    pm.fbs_msg.ee_analog_input = state.ee_AI();
    pm.fbs_msg.error_content = state.error_content();

    // Publish torque state
    pm.fbs_msg.joint_tor_average = state.joint_torque_average();
    pm.fbs_msg.joint_tor_min = state.joint_torque_min();
    pm.fbs_msg.joint_tor_max = state.joint_torque_max();
    pm.fbs_pub.publish(pm.fbs_msg);

    // Publish joint state
    pm.joint_msg.header.stamp = pm.fbs_msg.header.stamp;
    pm.joint_msg.position = pm.fbs_msg.joint_pos;
    pm.joint_msg.velocity = pm.fbs_msg.joint_vel;
    pm.joint_msg.effort = pm.fbs_msg.joint_tor;
    pm.joint_pub.publish(pm.joint_msg);

    // Publish tool pose
    //TmPoseConversion::msg_from_vec(pm.tool_pose_msg.pose, pm.fbs_msg.tool_pose);
    auto &pose = pm.fbs_msg.tool_pose;
    tf::Quaternion quat;
    quat.setRPY(pose[3], pose[4], pose[5]);
    tf::Transform Tbt(quat, tf::Vector3(pose[0], pose[1], pose[2]));
    tf::poseTFToMsg(Tbt, pm.tool_pose_msg.pose);
    pm.tool_pose_msg.header.stamp = pm.joint_msg.header.stamp;
    pm.tool_pose_msg.header.frame_id = base_frame_name_;
    /*pm.tool_pose_msg.pose.position.x = pose[0];
    pm.tool_pose_msg.pose.position.y = pose[1];
    pm.tool_pose_msg.pose.position.z = pose[2];
    pm.tool_pose_msg.pose.orientation.x = quat.x();
    pm.tool_pose_msg.pose.orientation.y = quat.y();
    pm.tool_pose_msg.pose.orientation.z = quat.z();
    pm.tool_pose_msg.pose.orientation.w = quat.w();*/
    pm.tool_pose_pub.publish(pm.tool_pose_msg);

    // Boardcast transform (tool pose)
    //TmPoseConversion::tf_from_vec(pm.transform, pm.fbs_msg.tool_pose);
    //pm.transform.setOrigin(tf::Vector3(pose[0], pose[1], pose[2]));
    //pm.transform.setRotation(quat);
    pm.tfbc.sendTransform(tf::StampedTransform(
        Tbt, pm.joint_msg.header.stamp, base_frame_name_, tool_frame_name_));
}
void TmRosNode::pub_data(){
  while(isRun){
    
    ethernetSlaveConnection->renew_all_data();

    publish_fbs();

    boost::this_thread::sleep_for(boost::chrono::milliseconds(15));
  }
}
void TmRosNode::publish_svr()
{
    PubMsg &pm = pm_;
    TmSvrData &data = iface_.svr.data;
    {
        boost::unique_lock<boost::mutex> lck(svr_mtx_);
        pm.svr_msg.id = data.transaction_id();
        pm.svr_msg.mode = (int)(data.mode());
        pm.svr_msg.content = std::string{ data.content(), data.content_len() };
        pm.svr_msg.error_code = (int)(data.error_code());
        svr_updated_ = true;
    }
    svr_cond_.notify_all();

    if ((int)(pm.svr_msg.error_code) != 0) {
        ROS_ERROR_STREAM("TM_ROS: (TM_SVR): MSG (" << pm.svr_msg.id << ") (" << (int)(pm.svr_msg.mode) << ") " << pm.svr_msg.content);  	
        ROS_ERROR_STREAM("TM_ROS: (TM_SVR) ROS Node Data Error" << (int)(pm.svr_msg.error_code));
    }
    else {
        ROS_INFO_STREAM("TM_ROS: (TM_SVR): MSG  (" << pm.svr_msg.id << ") (" << (int)(pm.svr_msg.mode) << ") " << pm.svr_msg.content);
    }    
    
    pm.svr_msg.header.stamp = ros::Time::now();
    pm.svr_pub.publish(pm.svr_msg);
}
void TmRosNode::svr_connect_recover()
{
    TmSvrCommunication &svr = iface_.svr;	
    int timeInterval = 0;
    int lastTimeInterval = 1000;
    	    	
    if (pub_reconnect_timeval_ms_ <= 0) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
    ROS_INFO_STREAM("TM_ROS: (TM_SVR): Reconnecting...");

    uint64_t startTimeMs = TmCommunication::get_current_time_in_ms();
    while (ros::ok() && timeInterval < pub_reconnect_timeval_ms_) {
        if ( lastTimeInterval/1000 != timeInterval/1000) {
            ROS_DEBUG_STREAM("TM_SVR reconnect remain : " << (0.001 * (pub_reconnect_timeval_ms_ - timeInterval)) << " sec... ");
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        lastTimeInterval = timeInterval;
        timeInterval = TmCommunication::get_current_time_in_ms() - startTimeMs;
    }    
    if (ros::ok() && pub_reconnect_timeval_ms_ >= 0) {
        ROS_DEBUG_STREAM("0 sec\nTM_ROS: (TM_SVR): connect" << (int)pub_reconnect_timeout_ms_ << "ms)...");
        svr.connect_socket("ethernet slave",pub_reconnect_timeout_ms_);
    }
}
void TmRosNode::sct_msg(TmSctData data)
{
    SctAndStaMsg &sm = sm_;

    sm.sct_msg.id = data.script_id();
    sm.sct_msg.script = std::string{ data.script(), data.script_len() };

    listenNodeConnection->check_is_on_listen_node_from_script(sm.sct_msg.id, sm.sct_msg.script);

    if (data.sct_has_error()) {
        ROS_ERROR_STREAM("TM_ROS: (TM_SCT): MSG : (" << sm.sct_msg.id << "): " << sm.sct_msg.script);
        ROS_ERROR_STREAM("TM_ROS: (TM_SCT):ROS Node Data Error: (" << (int)data.sct_has_error() << "): ");
    }
    else {
        ROS_INFO_STREAM("TM_ROS: (TM_SCT): MSG : (" << sm.sct_msg.id << "): " << sm.sct_msg.script);
    }

    sm.sct_msg.header.stamp = ros::Time::now();
    sm.sct_pub.publish(sm.sct_msg);
}

void TmRosNode::sta_msg(std::string subcmd, std::string subdata)
{
    SctAndStaMsg &sm = sm_;
    sm.sta_msg.subcmd = subcmd;
    sm.sta_msg.subdata = subdata;

    ROS_INFO_STREAM("TM_ROS: (TM_STA): res: (" << sm.sta_msg.subcmd << "): " << sm.sta_msg.subdata);

    sm.sta_msg.header.stamp = ros::Time::now();
    sm.sta_pub.publish(sm.sta_msg);

}