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
            rb = ethernetSlaveConnection->connect(t_o);
        }
        if (req.reconnect) {
            rb = ethernetSlaveConnection->re_connect(t_o,t_v);
        }
        else {
            ethernetSlaveConnection->no_connect();
        }
        break;
    case tm_msgs::ConnectTMRequest::TMSCT:
        rb = listenNodeConnection->connect_tmsct(req.timeout, req.timeval, req.connect, req.reconnect);
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
    bool rb = listenNodeConnection->send_listen_node_script(req.id, req.script);
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
    res.ok = listenNodeConnection->ask_sta_struct(req.subcmd, req.subdata, req.wait_time, res.subcmd, res.subdata);
    return res.ok;
}

