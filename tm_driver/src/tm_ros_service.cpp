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
            rb = iface_.svr.start(t_o);
        }
        if (req.reconnect) {
            pub_reconnect_timeout_ms_ = t_o;
            pub_reconnect_timeval_ms_ = t_v;
            print_info("TM_ROS: set SVR reconnect timeout %dms, timeval %dms", t_o, t_v);
        }
        else {
            // no reconnect
            pub_reconnect_timeval_ms_ = -1;
            print_info("TM_ROS: set SVR NOT reconnect");
        }
        break;
    case tm_msgs::ConnectTMRequest::TMSCT:
        if (req.connect) {
            print_info("TM_ROS: (re)connect(%d) TM_SCT", t_o);
            iface_.sct.halt();
            rb = iface_.sct.start(t_o);
        }
        if (req.reconnect) {
            sct_reconnect_timeout_ms_ = t_o;
            sct_reconnect_timeval_ms_ = t_v;
            print_info("TM_ROS: set SCT reconnect timeout %dms, timeval %dms", t_o, t_v);
        }
        else {
            // no reconnect
            sct_reconnect_timeval_ms_ = -1;
            print_info("TM_ROS: set SCT NOT reconnect");
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
    TmSvrData &data = iface_.svr.data;
    bool rb = false;

    svr_updated_ = false;

    rb = (iface_.svr.send_content(req.id, TmSvrData::Mode::READ_STRING, req.item) == iface_.RC_OK);
    if (!rb) return rb;

    if (req.wait_time > 0.0) {
        boost::mutex lock;
        boost::unique_lock<boost::mutex> locker(lock);
        if (!svr_updated_) {
            svr_cond_.wait_for(locker, boost::chrono::duration<double>(req.wait_time));
        }
        if (svr_updated_) {
            res.id = data.transaction_id();
            res.value = std::string{ data.content(), data.content_len() };
            svr_updated_ = false;
        }
        else rb = false;
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

bool TmRosNode::ask_sta(tm_msgs::AskStaRequest &req, tm_msgs::AskStaResponse &res)
{
    TmStaData &data = iface_.sct.sta_data;
    bool rb = false;

    sta_updated_ = false;

    rb = (iface_.sct.send_sta_request(req.subcmd, req.subdata) == iface_.RC_OK);

    if (req.wait_time > 0.0) {
        boost::mutex lock;
        boost::unique_lock<boost::mutex> locker(lock);
        if (!sta_updated_) {
            svr_cond_.wait_for(locker, boost::chrono::duration<double>(req.wait_time));
        }
        if (sta_updated_) {
            res.subcmd = data.subcmd_str();
            res.subdata = std::string{ data.subdata(), data.subdata_len() };
            sta_updated_ = false;
        }
        else rb = false;
    }
    res.ok = rb;
    return rb;
}
