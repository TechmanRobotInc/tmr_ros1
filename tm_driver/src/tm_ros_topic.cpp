#include "tm_driver/tm_ros_node.h"

////////////////////////////////
// Topic
////////////////////////////////

void TmRosNode::publish_fbs(TmCommRC rc)
{
    PubMsg &pm = pm_;
    TmRobotState &state = iface_.state;

    // Publish feedback state
    pm.fbs_msg.header.stamp = ros::Time::now();
    if(rc != TmCommRC::TIMEOUT){
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

bool TmRosNode::publish_func()
{
    TmSvrCommunication &svr = iface_.svr;
    int n;
    auto rc = svr.recv_spin_once(1000, &n);

    if (rc == TmCommRC::ERR ||
        rc == TmCommRC::NOTREADY ||
        rc == TmCommRC::NOTCONNECT) {
        return false;
    }
    bool fbs = false;
    std::vector<TmPacket> &pack_vec = svr.packet_list();

    for (auto &pack : pack_vec) {
        if (pack.type == TmPacket::Header::CPERR) {
            svr.tmSvrErrData.set_CPError(pack.data.data(), pack.data.size());
            ROS_ERROR_STREAM("TM_ROS: (TM_SVR) ROS Node Header CPERR" << (int)svr.tmSvrErrData.error_code());
        }
        else if (pack.type == TmPacket::Header::TMSVR) {

            if (svr.data.is_valid() && (svr.data.mode()== TmSvrData::Mode::BINARY))
            {
                if ((int)iface_.svr.tmSvrErrData.error_code())  {} 
                else
                {
                    svr.tmSvrErrData.error_code(TmCPError::Code::Ok); 
                }
            }
            else if (svr.data.is_valid() && (svr.data.mode() == TmSvrData::Mode::RESPONSE))
            {
                if ((int)iface_.svr.data.error_code() != 0)  {}
                else
                {  
                  svr.tmSvrErrData.error_code(TmCPError::Code::Ok);
                }
            }            
            else
            {
                svr.tmSvrErrData.error_code(TmCPError::Code::Ok); 
            }            
            TmSvrData::build_TmSvrData(svr.data, pack.data.data(), pack.data.size(), TmSvrData::SrcType::Shallow);

            if (svr.data.is_valid()) {
                switch (svr.data.mode()) {
                case TmSvrData::Mode::RESPONSE:
                    //print_info("TM_ROS: (TM_SVR): (%s) RESPONSE [%d]",
                    //    svr.data.transaction_id().c_str(), (int)(svr.data.error_code()));
                    publish_svr();
                    break;
                case TmSvrData::Mode::BINARY:
                    svr.state.mtx_deserialize(svr.data.content(), svr.data.content_len());
                    fbs = true;
                    break;
                case TmSvrData::Mode::READ_STRING:
                case TmSvrData::Mode::READ_JSON:
                    publish_svr();
                    break;
                default:
                    ROS_INFO_STREAM("TM_ROS: (TM_SVR): (" <<
                        svr.data.transaction_id() << "): invalid mode (" << (int)(svr.data.mode()) << ")");
                    break;
                }
            }
            else {
                ROS_ERROR_STREAM("TM_ROS: (TM_SVR): invalid data");
            }
        }
        else {
            ROS_ERROR_STREAM("TM_ROS: (TM_SVR): invalid header");
        }
    }
    if (fbs) {
        publish_fbs(rc);
    }
    if(rc == TmCommRC::TIMEOUT){
      ROS_INFO_STREAM_ONCE( "TM_ROS: (TM_SVR): lINK TIMEOUT");
      return false;
    }
    return true;
}
void TmRosNode::cq_monitor(){  //Connection quality
    diconnectTimes ++;
    initialNotConnectTime =  TmCommunication::get_current_time_in_ms();
}
void TmRosNode::cq_manage(){
    
    notConnectTimeInS = (TmCommunication::get_current_time_in_ms() - initialNotConnectTime)/1000;
    if(diconnectTimes == 0){
        return;
    }
    if(notConnectTimeInS>maxNotConnectTimeInS){
        maxNotConnectTimeInS = notConnectTimeInS;
    }
}
bool TmRosNode::rc_halt(){  //Stop rescue connection
    if(maxTrialTimeInMinute == -1){
        return false;
    }
    if((int)(notConnectTimeInS/60) >= maxTrialTimeInMinute){
      print_info("TM_ROS: (TM_SVR): notConnectTimeInS = (%d) , maxTrialTimeInMinute = (%d)", 
          (int)notConnectTimeInS, (int)maxTrialTimeInMinute);
        return true;
    }
    return false;
}
void TmRosNode::publisher()
{
    PubMsg &pm = pm_;
    TmSvrCommunication &svr = iface_.svr;

    ROS_INFO_STREAM("TM_ROS: publisher thread begin");
    initialNotConnectTime =  TmCommunication::get_current_time_in_ms();
    //PubMsg pm;
    //pm.fbs_pub = nh_.advertise<tm_msgs::FeedbackState>("feedback_states", 1);
    //pm.joint_pub = nh_.advertise<sensor_msgs::JointState>("joint_states", 1);
    //pm.tool_pose_pub = nh_.advertise<geometry_msgs::PoseStamped>("tool_pose", 1);

    //pm.svr_pub = nh_.advertise<tm_msgs::SvrResponse>("tm_driver/svr_response", 1);

    pm.joint_msg.name = joint_names_;
    pm.joint_msg.position.assign(joint_names_.size(), 0.0);
    pm.joint_msg.velocity.assign(joint_names_.size(), 0.0);
    pm.joint_msg.effort.assign(joint_names_.size(), 0.0);

    while (ros::ok()) {
        //bool reconnect = false;
        if (svr_recovery_is_halt) {
            boost::this_thread::sleep_for(boost::chrono::milliseconds(1));
        }
        else   
        {	        	
            if (!svr.recv_init()) {
                ROS_INFO_STREAM("TM_ROS: (TM_SVR): is not connected");
                cq_manage();
                publish_fbs(TmCommRC::TIMEOUT);
                if(rc_halt()) {
                    ROS_FATAL_STREAM("TM_ROS: (TM_SVR): Ethernet slave connection stopped");
                    ROS_FATAL_STREAM("TM_ROS: (TM_SVR): LinkLost = " << (int)pm.fbs_msg.disconnection_times << ", MaxLostTime(s) = " << (int)pm.fbs_msg.max_not_connect_in_s);
                    svr_recovery_is_halt = true;
                    svr.close_socket();
                }
            }
            if (!svr_recovery_is_halt) {
                while (ros::ok() && svr.is_connected()) {
                    if (!publish_func()){
                        cq_monitor();
                        break;
                    }
                }
                        
                svr.close_socket();
                if (!ros::ok()) break;
                svr_connect_recover();
            }
        } 
    }
    svr.close_socket();
    ROS_INFO_STREAM("TM_ROS: publisher thread end");
}

void TmRosNode::svr_connect_recover()
{
    TmSvrCommunication &svr = iface_.svr;	
    int timeInterval = 0;
    int lastTimeInterval = 1000;
    	    	
    if (pub_reconnect_timeval_ms_ <= 0) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
    print_info("TM_ROS: (TM_SVR): reconnect in ");

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
        svr.connect_socket(pub_reconnect_timeout_ms_);
    }
}

void TmRosNode::check_is_on_listen_node_from_script(std::string id, std::string script){
    std::string idzero = "0";
    std::string ok = "OK";
    std::string errorString = "ERROR";
    if(idzero.compare(id)==0 && errorString.compare(script)!=0 &&  ok.compare(script)!=0){
        iface_.back_to_listen_node();
    }
}

void TmRosNode::sct_msg()
{
    SctAndStaMsg &sm = sm_;
    TmSctData &data = iface_.sct.sct_data;

    sm.sct_msg.id = data.script_id();
    sm.sct_msg.script = std::string{ data.script(), data.script_len() };

    check_is_on_listen_node_from_script(sm.sct_msg.id, sm.sct_msg.script);

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

void TmRosNode::sta_msg()
{
    SctAndStaMsg &sm = sm_;
    TmStaData &data = iface_.sct.sta_data;
    {
        boost::lock_guard<boost::mutex> lck(sta_mtx_);
        sm.sta_msg.subcmd = data.subcmd_str();
        sm.sta_msg.subdata = std::string{ data.subdata(), data.subdata_len() };
        sta_updated_ = true;
    }
    sta_cond_.notify_all();

    ROS_INFO_STREAM("TM_ROS: (TM_STA): res: (" << sm.sta_msg.subcmd << "): " << sm.sta_msg.subdata);

    sm.sta_msg.header.stamp = ros::Time::now();
    sm.sta_pub.publish(sm.sta_msg);
}

bool TmRosNode::sct_func()
{
    TmSctCommunication &sct = iface_.sct;
    int n;
    firstCheckIsOnListenNodeCondVar.notify_one();
    
    auto rc = sct.recv_spin_once(1000, &n);
    if (rc == TmCommRC::ERR ||
        rc == TmCommRC::NOTREADY ||
        rc == TmCommRC::NOTCONNECT) {
        return false;
    }
    else if (rc != TmCommRC::OK) {
        return true;
    }
    std::vector<TmPacket> &pack_vec = sct.packet_list();

    for (auto &pack : pack_vec) {
        switch (pack.type) {
        case TmPacket::Header::CPERR:
            sct.tmSctErrData.set_CPError(pack.data.data(), pack.data.size());
            ROS_ERROR_STREAM("TM_ROS: (TM_SCT) ROS Node Header CPERR" << (int)sct.tmSctErrData.error_code());
            break;

        case TmPacket::Header::TMSCT:

            sct.tmSctErrData.error_code(TmCPError::Code::Ok);

            //TODO ? lock and copy for service response
            TmSctData::build_TmSctData(sct.sct_data, pack.data.data(), pack.data.size(), TmSctData::SrcType::Shallow);

            sct_msg();
            break;

        case TmPacket::Header::TMSTA:

            sct.tmSctErrData.error_code(TmCPError::Code::Ok);

            TmStaData::build_TmStaData(sct.sta_data, pack.data.data(), pack.data.size(), TmStaData::SrcType::Shallow);

            sta_msg();
            break;

        default:
            ROS_ERROR_STREAM("TM_ROS: (TM_SCT): invalid header");
            break;
        }
    }
    return true;
}

void TmRosNode::check_is_on_listen_node(){
    std::unique_lock<std::mutex> firstCheckIsOnListenNodeLock(firstCheckIsOnListenNodeMutex);
    std::unique_lock<std::mutex> checkIsOnListenNodeLock(checkIsOnListenNodeMutex);
    
    firstCheckIsOnListenNodeCondVar.wait(firstCheckIsOnListenNodeLock);
    std::this_thread::sleep_for(std::chrono::milliseconds(2));

    while (ros::ok()){
        std::string reSubcmd;
        std::string reSubdata;
        ask_sta_struct("00","",1,reSubcmd,reSubdata);
        bool isInListenNode = false;

        std::istringstream(reSubdata) >> std::boolalpha >> isInListenNode;
    
        if(isInListenNode){
            ROS_INFO_STREAM("On listen node !");
            iface_.back_to_listen_node();
        } else{
            ROS_INFO_STREAM("Not on listen node !");
        }
        checkIsOnListenNodeCondVar.wait(checkIsOnListenNodeLock);
    }
}

void TmRosNode::sct_responsor()
{
    SctAndStaMsg &sm = sm_;
    TmSctCommunication &sct = iface_.sct;

    boost::this_thread::sleep_for(boost::chrono::milliseconds(50));

    ROS_INFO_STREAM("TM_ROS: sct_response thread begin");

    //SctMsg sm;
    //sm.sct_pub = nh_.advertise<tm_msgs::SctResponse>("tm_driver/sct_response", 1);
    //sm.sta_pub = nh_.advertise<tm_msgs::StaResponse>("tm_driver/sta_response", 1);

    while (ros::ok()) {
        //bool reconnect = false;
        if (!sct.recv_init()) {
            ROS_INFO_STREAM("TM_ROS: (TM_SCT): is not connected");
        }
        firstEnter = true;
        while (ros::ok() && sct.is_connected() && iface_.svr.is_connected()) {
            if(firstEnter){
                checkIsOnListenNodeCondVar.notify_one();
                firstEnter = false;
            }
            if (!sct_func()) break;
        }
        sct.close_socket();
        if (!ros::ok()) break;
        sct_connect_recover();
    }
    checkIsOnListenNodeCondVar.notify_one();
    firstCheckIsOnListenNodeCondVar.notify_one();
    
    sct.close_socket();
    ROS_INFO_STREAM("TM_ROS: sct_response thread end");		
}

void TmRosNode::sct_connect_recover()
{
    TmSctCommunication &sct = iface_.sct;
    int timeInterval = 0;
    int lastTimeInterval=1000;
            	
    if (sct_reconnect_timeval_ms_ <= 0) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
    ROS_INFO_STREAM("TM_ROS: (TM_SCT) reconnect in ");

    uint64_t startTimeMs = TmCommunication::get_current_time_in_ms();
    while (ros::ok() && timeInterval < sct_reconnect_timeval_ms_) {
        if ( lastTimeInterval/1000 != timeInterval/1000) {
            ROS_DEBUG_STREAM("TM_SCT reconnect remain : " << (0.001 * (sct_reconnect_timeval_ms_ - timeInterval)) << " sec... ");
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        lastTimeInterval = timeInterval;
        timeInterval = TmCommunication::get_current_time_in_ms() - startTimeMs;
    }
    if (ros::ok() && sct_reconnect_timeval_ms_ >= 0) {
        ROS_DEBUG_STREAM("0 sec\nTM_ROS: (TM_SCT) connect(" << (int)sct_reconnect_timeout_ms_ << "ms)...");
        sct.connect_socket(sct_reconnect_timeout_ms_);
    }
}
