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
            print_info("TM_ROS: (re)connect(%d) TMSVR", t_o);
            iface_.svr.halt();
            rb = iface_.svr.start(t_o);
        }
        if (req.reconnect) {
            pub_reconnect_timeout_ms_ = t_o;
            pub_reconnect_timeval_ms_ = t_v;
            print_info("TM_ROS: set TMSVR reconnect timeout %dms, timeval %dms", t_o, t_v);
        }
        else {
            // no reconnect
            pub_reconnect_timeval_ms_ = -1;
            print_info("TM_ROS: set TMSVR NOT reconnect");
        }
        break;
    case tm_msgs::ConnectTMRequest::TMSCT:
        if (req.connect) {
            print_info("TM_ROS: (re)connect(%d) TMSCT", t_o);
            iface_.sct.halt();
            rb = iface_.sct.start(t_o);
        }
        if (req.reconnect) {
            iface_.sct.set_reconnect_timeout(t_o);
            iface_.sct.set_reconnect_timeval(t_v);
            print_info("TM_ROS: set TMSCT reconnect timeout %dms, timeval %dms", t_o, t_v);
        }
        else {
            // no reconnect
            iface_.sct.set_reconnect_timeval(-1);
            print_info("TM_ROS: set TMSCT NOT reconnect");
        }
        break;
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
    switch (req.event_type) {
    case tm_msgs::SetEventRequest::EVENT_STOP:
        rb = iface_.set_stop();
        break;
    case tm_msgs::SetEventRequest::EVENT_PAUSE:
        rb = iface_.set_pause();
        break;
    case tm_msgs::SetEventRequest::EVENT_RESUME:
        rb = iface_.set_resume();
        break;
    }
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
