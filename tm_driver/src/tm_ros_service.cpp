#include "tm_driver/tm_ros_node.h"

////////////////////////////////
// Service
////////////////////////////////

bool TmRosNode::connect_tm(tm_msgs::ConnectTMRequest &req, tm_msgs::ConnectTMResponse &res)
{
    bool rb = true;
    int t_o = (int)(1000.0 * req.timeout);
    int t_v = (int)(1000.0 * req.timeval);
    switch (req.server) {
    case tm_msgs::ConnectTMRequest::TMSVR:
        if (req.connect) {
            print_info("TM_ROS: (re)connect(%d) TM_SVR", t_o);
            iface_.svr.halt();
            rb = iface_.svr.start_tm_svr(t_o);
        }
        if (req.reconnect) {
            if (connect_recovery_is_halt)
            {
                pub_reconnect_timeout_ms_ = 1000;
                pub_reconnect_timeval_ms_ = 3000;
                initialNotConnectTime =  TmCommunication::get_current_time_in_ms();
                notConnectTimeInS = 0;
                diconnectTimes = 0;
                maxNotConnectTimeInS = 0;
                connect_recovery_is_halt = false;
                rb = iface_.svr.start_tm_svr(5000);
                ROS_INFO_STREAM("TM_ROS: TM_SVR resume connection recovery");
            }
            else
            {
                pub_reconnect_timeout_ms_ = t_o;
                pub_reconnect_timeval_ms_ = t_v;
            }	
            ROS_INFO_STREAM("TM_ROS: set TM_SVR reconnect timeout " << (int)pub_reconnect_timeout_ms_ << "ms, timeval " << (int)pub_reconnect_timeval_ms_ << "ms");
        }
        else {
            // no reconnect
            pub_reconnect_timeval_ms_ = -1;
            ROS_INFO_STREAM("TM_ROS: set TM_SVR NOT reconnect");
        }
        break;
    case tm_msgs::ConnectTMRequest::TMSCT:
        if (req.connect) {
            ROS_INFO_STREAM("TM_ROS: (re)connect(" << (int)t_o << ") TM_SCT");
            iface_.sct.halt();
            rb = iface_.sct.start_tm_sct(t_o);
        }
        if (req.reconnect) {
            if (connect_recovery_is_halt)
            {
                sct_reconnect_timeout_ms_ = 1000;
                sct_reconnect_timeval_ms_ = 3000;
                connect_recovery_is_halt = false;
                rb = iface_.sct.start_tm_sct(5000);
                ROS_INFO_STREAM("TM_ROS: TM_SCT resume connection recovery");                					
            }
            else
            {				
                sct_reconnect_timeout_ms_ = t_o;
                sct_reconnect_timeval_ms_ = t_v;
            }			
            ROS_INFO_STREAM("TM_ROS: set TM_SCT reconnect timeout " << (int)t_o << "ms, timeval " << (int)t_v << "ms");
        }
        else {
            // no reconnect
            sct_reconnect_timeval_ms_ = -1;
            ROS_INFO_STREAM("TM_ROS: set TM_SCT NOT reconnect");
        }
        break;
    }
    res.ok = rb;
    return rb;
}

bool TmRosNode::write_item(tm_msgs::WriteItemRequest &req, tm_msgs::WriteItemResponse &res)
{
    bool rb = false;
    std::string content = req.item + "=" + req.value;
    rb = (iface_.svr.send_content_str(req.id, content) == iface_.RC_OK);
    res.ok = rb;
    return rb;
}

bool TmRosNode::ask_item(tm_msgs::AskItemRequest &req, tm_msgs::AskItemResponse &res)
{
    PubMsg &pm = pm_;
    TmSvrData &data = iface_.svr.data;
    bool rb = false;

    svr_mtx_.lock();
    svr_updated_ = false;
    svr_mtx_.unlock();

    rb = (iface_.svr.send_content(req.id, TmSvrData::Mode::READ_STRING, req.item) == iface_.RC_OK);

    {
        boost::unique_lock<boost::mutex> lck(svr_mtx_);
        if (rb && req.wait_time > 0.0) {
            if (!svr_updated_) {
                svr_cond_.wait_for(lck, boost::chrono::duration<double>(req.wait_time));
            }
            if (!svr_updated_) {
                rb = false;
            }
            res.id = pm.svr_msg.id;
            res.value = pm.svr_msg.content;
        }
        svr_updated_ = false;
    }
    res.ok = rb;
    return rb;
}

bool TmRosNode::send_script(tm_msgs::SendScriptRequest &req, tm_msgs::SendScriptResponse &res)
{
    bool rb = (iface_.sct.send_script_str(req.id, req.script) == iface_.RC_OK);
    res.ok = rb;
    return rb;
}

bool TmRosNode::set_event(tm_msgs::SetEventRequest &req, tm_msgs::SetEventResponse &res)
{
    bool rb = false;
    switch (req.func) {
    case tm_msgs::SetEventRequest::EXIT:
        rb = iface_.script_exit();
        break;
    case tm_msgs::SetEventRequest::TAG:
        rb = iface_.set_tag((int)(req.arg0), (int)(req.arg1));
        break;
    case tm_msgs::SetEventRequest::WAIT_TAG:
        rb = iface_.set_wait_tag((int)(req.arg0), (int)(req.arg1));
        break;
    case tm_msgs::SetEventRequest::STOP:
        rb = iface_.set_stop();
        break;
    case tm_msgs::SetEventRequest::PAUSE:
        rb = iface_.set_pause();
        break;
    case tm_msgs::SetEventRequest::RESUME:
        rb = iface_.set_resume();
        break;
    }
    res.ok = rb;
    return rb;
}

bool TmRosNode::set_io(tm_msgs::SetIORequest &req, tm_msgs::SetIOResponse &res)
{
    bool rb = iface_.set_io(TmIOModule(req.module), TmIOType(req.type), int(req.pin), req.state);
    res.ok = rb;
    return rb;
}

bool TmRosNode::set_positions(tm_msgs::SetPositionsRequest &req, tm_msgs::SetPositionsResponse &res)
{
    bool rb = false;
    switch(req.motion_type) {
    case tm_msgs::SetPositionsRequest::PTP_J:
        rb = iface_.set_joint_pos_PTP(req.positions, req.velocity, req.acc_time, req.blend_percentage, req.fine_goal);
        break;
    case tm_msgs::SetPositionsRequest::PTP_T:
        rb = iface_.set_tool_pose_PTP(req.positions, req.velocity, req.acc_time, req.blend_percentage, req.fine_goal);
        break;
    case tm_msgs::SetPositionsRequest::LINE_T:
        rb = iface_.set_tool_pose_Line(req.positions, req.velocity, req.acc_time, req.blend_percentage, req.fine_goal);
        break;
    }
    res.ok = rb;
    return rb;
}

bool TmRosNode::ask_sta_struct(std::string subcmd, std::string subdata, double waitTime,std::string &reSubcmd, std::string &reSubdata)
{
    SctAndStaMsg &sm = sm_;
    TmStaData &data = iface_.sct.sta_data;
    bool rb = false;

    sta_mtx_.lock();
    sta_updated_ = false;
    sta_mtx_.unlock();

    rb = (iface_.sct.send_sta_request(subcmd, subdata) == iface_.RC_OK);

    {
        boost::unique_lock<boost::mutex> lck(sta_mtx_);
        if (rb && waitTime > 0.0) {
            if (!sta_updated_) {
                sta_cond_.wait_for(lck, boost::chrono::duration<double>(waitTime));
            }
            if (!sta_updated_) {
                rb = false;
            }
            reSubcmd = sm.sta_msg.subcmd;
            reSubdata = sm.sta_msg.subdata;
        }
        sta_updated_ = false;
    }

    return rb;
}

bool TmRosNode::ask_sta(tm_msgs::AskStaRequest &req, tm_msgs::AskStaResponse &res)
{
    res.ok = ask_sta_struct(req.subcmd, req.subdata, req.wait_time, res.subcmd, res.subdata);
    return res.ok;
}

