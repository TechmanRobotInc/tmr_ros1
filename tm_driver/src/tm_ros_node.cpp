
#include "tm_driver/tm_ros_node.h"


////////////////////////////////
// Node
////////////////////////////////

TmRosNode::TmRosNode(const std::string &host)
    //: iface_(host, nullptr, &sct_cv_)
    : iface_(host, nullptr, nullptr)
    , as_(nh_, "joint_trajectory_action"
        , boost::bind(&TmRosNode::goalCB, this, _1)
        , boost::bind(&TmRosNode::cancelCB, this, _1)
        , false)
    , has_goal_(false)
{
    ////////////////////////////////
    // Param.
    ////////////////////////////////
    std::string prefix = "";
    if (ros::param::get("~prefix", prefix)) {
        if (prefix.length())
            print_info("TM_ROS: set prefix to %s", prefix.c_str());
    }
    joint_names_.push_back(prefix + "shoulder_1_joint");
    joint_names_.push_back(prefix + "shoulder_2_joint");
    joint_names_.push_back(prefix + "elbow_joint");
    joint_names_.push_back(prefix + "wrist_1_joint");
    joint_names_.push_back(prefix + "wrist_2_joint");
    joint_names_.push_back(prefix + "wrist_3_joint");

    std::string frame_name = "base";
    if (ros::param::get("~base_frame", frame_name)) {
        print_info("TM_ROS: set base_frame to %s", frame_name.c_str());
    }
    base_frame_name_ = prefix + frame_name;
    frame_name = "tool0";
    if (ros::param::get("~tool_frame", frame_name)) {
        print_info("TM_ROS: set tool_frame to %s", frame_name.c_str());
    }
    tool_frame_name_ = prefix + frame_name;

    bool auto_stick_play = true;
    if (ros::param::get("~auto_stick_play", auto_stick_play)) {
        print_info("TM_ROS: set auto_stick_play to %d", auto_stick_play);
    }

    ////////////////////////////////
    // TmDriver
    ////////////////////////////////
    iface_.start(5000, auto_stick_play);

    ////////////////////////////////
    // Topic
    ////////////////////////////////

    pm_.fbs_pub = nh_.advertise<tm_msgs::FeedbackState>("feedback_states", 1);
    pm_.joint_pub = nh_.advertise<sensor_msgs::JointState>("joint_states", 1);
    pm_.tool_pose_pub = nh_.advertise<geometry_msgs::PoseStamped>("tool_pose", 1);

    pm_.svr_pub = nh_.advertise<tm_msgs::SvrResponse>("tm_driver/svr_response", 1);

    sm_.sct_pub = nh_.advertise<tm_msgs::SctResponse>("tm_driver/sct_response", 1);
    sm_.sta_pub = nh_.advertise<tm_msgs::StaResponse>("tm_driver/sta_response", 1);

    svr_updated_ = false;
    pub_reconnect_timeout_ms_ = 1000;
    pub_reconnect_timeval_ms_ = 3000;
    pub_thread_ = boost::thread(boost::bind(&TmRosNode::publisher, this));

    sta_updated_ = false;
    sct_reconnect_timeout_ms_ = 1000;
    sct_reconnect_timeval_ms_ = 3000;
    sct_thread_ = boost::thread(boost::bind(&TmRosNode::sct_responsor, this));

    ////////////////////////////////
    // Action
    ////////////////////////////////
    as_.start();

    ////////////////////////////////
    // Service
    ////////////////////////////////
    connect_srv_ = nh_.advertiseService("tm_diver/connect_tm", &TmRosNode::connect_tm, this);

    write_item_srv_ = nh_.advertiseService("tm_driver/write_item", &TmRosNode::write_item, this);
    ask_item_srv_ = nh_.advertiseService("tm_driver/ask_item", &TmRosNode::ask_item, this);

    send_script_srv_ = nh_.advertiseService("tm_driver/send_script", &TmRosNode::send_script, this);

    set_event_srv_ = nh_.advertiseService("tm_driver/set_event", &TmRosNode::set_event, this);
    set_io_srv_ = nh_.advertiseService("tm_driver/set_io", &TmRosNode::set_io, this);

    set_positions_srv_ = nh_.advertiseService("tm_driver/set_positions", &TmRosNode::set_positions, this);

    ask_sta_srv_ = nh_.advertiseService("tm_driver/ask_sta", &TmRosNode::ask_sta, this);

}
TmRosNode::~TmRosNode()
{
    halt();
}
void TmRosNode::halt()
{
    printf("TM_ROS: halt\n");
    sta_updated_ = true;
    sta_cond_.notify_all();
    svr_updated_ = true;
    svr_cond_.notify_all();
    if (sct_thread_.joinable()) { sct_thread_.join(); }
    if (pub_thread_.joinable()) { pub_thread_.join(); }
    // Driver
    iface_.halt();
}

////////////////////////////////
// Action
////////////////////////////////

// helper function

bool TmRosNode::has_points(const trajectory_msgs::JointTrajectory &traj)
{
    if (traj.points.size() == 0) return false;
    for (auto &point : traj.points) {
        if (point.positions.size() != traj.joint_names.size() ||
            point.velocities.size() != traj.joint_names.size())
            return false;
    }
    return true;
}
bool TmRosNode::has_limited_velocities(const trajectory_msgs::JointTrajectory &traj)
{
    return true;
}
bool TmRosNode::is_traj_finite(const trajectory_msgs::JointTrajectory &traj)
{
    for (size_t i = 0; i < traj.points.size(); ++i) {
        for (size_t j = 0; j < traj.points[i].positions.size(); ++j) {
            if (!std::isfinite(traj.points[i].positions[j]))
                return false;
            if (!std::isfinite(traj.points[i].velocities[j]))
                return false;
        }
    }
    return true;
}
void TmRosNode::reorder_traj_joints(trajectory_msgs::JointTrajectory& traj) {
    /* Reorders trajectory - destructive */
    std::vector<size_t> mapping;
    mapping.resize(joint_names_.size(), joint_names_.size());
    for (size_t i = 0; i < traj.joint_names.size(); ++i) {
        for (size_t j = 0; j < joint_names_.size(); ++j) {
            if (traj.joint_names[i] == joint_names_[j])
                mapping[j] = i;
        }
    }
    std::vector<trajectory_msgs::JointTrajectoryPoint> new_traj;
    for (unsigned int i = 0; i < traj.points.size(); ++i) {
        trajectory_msgs::JointTrajectoryPoint new_point;
        for (unsigned int j = 0; j < traj.points[i].positions.size(); ++j) {
            new_point.positions.push_back(traj.points[i].positions[mapping[j]]);
            new_point.velocities.push_back(traj.points[i].velocities[mapping[j]]);
            if (traj.points[i].accelerations.size() != 0)
                new_point.accelerations.push_back(traj.points[i].accelerations[mapping[j]]);
        }
        new_point.time_from_start = traj.points[i].time_from_start;
        new_traj.push_back(new_point);
    }
    traj.points = new_traj;
}
bool TmRosNode::is_start_positions_match(const trajectory_msgs::JointTrajectory &traj, double eps)
{
    auto q_act = iface_.state.mtx_joint_angle();

    for (size_t i = 0; i < traj.points[0].positions.size(); ++i) {
        if (fabs(traj.points[0].positions[i] - q_act[i]) > eps)
            return false;
    }
    return true;
}
void TmRosNode::set_result(int32_t err_code, const std::string &err_str)
{
    result_.error_code = err_code;
    result_.error_string = err_str;
    if (err_code != result_.SUCCESSFUL && err_str.length()) {
        print_error(err_str.c_str());
    }
}

void TmRosNode::set_pvt_traj(TmPvtTraj &pvts, const trajectory_msgs::JointTrajectory &traj)
{
    size_t begin_index = 0;
    //pvts.time_vec.clear();
    //pvts.positions_vec.clear();
    //pvts.velocities_vec.clear();

    TmPvtPoint point;

    pvts.mode = TmPvtMode::Joint;

    if (traj.points[0].time_from_start.toSec() != 0.0) {
        print_warn("Trajectory's first point should be the current position, with time_from_start set to 0.0");
        /*pvts.time_vec.push_back(traj.points[0].time_from_start.toSec());
        pvts.positions_vec.push_back(traj.points[0].positions);
        pvts.velocities_vec.push_back(traj.points[0].velocities);*/
        point.time = traj.points[0].time_from_start.toSec();
        point.positions = traj.points[0].positions;
        point.velocities = traj.points[0].velocities;
        pvts.points.push_back(point);
    }
    for (size_t i = 1; i < traj.points.size(); ++i) {
        /*double t = traj.points[i].time_from_start.toSec() - traj.points[i - 1].time_from_start.toSec();
        pvts.time_vec.push_back(t);
        pvts.positions_vec.push_back(traj.points[i].positions);
        pvts.velocities_vec.push_back(traj.points[i].velocities);*/
        point.time = traj.points[i].time_from_start.toSec() - traj.points[i - 1].time_from_start.toSec();
        point.positions = traj.points[i].positions;
        point.velocities = traj.points[i].velocities;
        pvts.points.push_back(point);
    }
    pvts.total_time = traj.points.back().time_from_start.toSec();
}

void TmRosNode::traj_action(TmPvtTraj pvts)
{
    print_info("TM_ROS: trajectory thread begin");
    iface_.run_pvt_traj(pvts);
    if (has_goal_) {
        result_.error_code = result_.SUCCESSFUL;
        gh_.setSucceeded(result_);
        has_goal_ = false;
    }
    print_info("TM_ROS: trajectory thread end");
}

// action function

void TmRosNode::goalCB(actionlib::ServerGoalHandle<control_msgs::FollowJointTrajectoryAction> gh)
{
    print_info("TM_ROS: on goal");

    //
    // check robot
    //

    if (!iface_.sct.is_connected()) {
        set_result(-100, "TM_ROS: Cannot accept new trajectories. TM_SCT is not connected");
        gh.setRejected(result_, result_.error_string);
        return;
    }
    if (iface_.state.has_error()) {
        set_result(-100, "TM_ROS: Cannot accept new trajectories. Robot has error");
        gh.setRejected(result_, result_.error_string);
        return;
    }

    //
    // check goal
    //

    //actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction>::Goal goal;
    auto goal = *(gh.getGoal());

    gh_ = gh;

    if (has_goal_) {
        print_warn("TM_ROS: Received new goal while still executing previous trajectory. Canceling previous trajectory");
        set_result(-100, "TM_ROS: Received another trajectory");
        gh_.setAborted(result_, result_.error_string);

        boost::this_thread::sleep_for(boost::chrono::milliseconds(250));
    }
    if (!has_points(goal.trajectory)) {
        set_result(result_.INVALID_GOAL, "TM_ROS: Received a goal without points");
        return;
    }
    if (!has_limited_velocities(goal.trajectory)) {
        set_result(result_.INVALID_GOAL, "TM_ROS: Received a goal with velocities that are higher than max velocity");
        return;
    }
    if (!is_traj_finite(goal.trajectory)) {
        set_result(result_.INVALID_GOAL, "TM_ROS: Received a goal with infinities or NaNs");
        return;
    }
    reorder_traj_joints(goal.trajectory);

    if (!is_start_positions_match(goal.trajectory, 0.01)) {
        set_result(result_.INVALID_GOAL, "TM_ROS: Start point doesn't match current pose");
        return;
    }
    //print_traj(goal.trajectory);

    gh_.setAccepted();

    has_goal_ = true;

    TmPvtTraj pvts;

    set_pvt_traj(pvts, goal.trajectory);

    boost::thread(boost::bind(&TmRosNode::traj_action, this, pvts)).detach();
}

void TmRosNode::cancelCB(actionlib::ServerGoalHandle<control_msgs::FollowJointTrajectoryAction> gh)
{
    print_info("TM_ROS: on cancel");

    if (has_goal_) {
        if (gh == gh_) {
            iface_.stop_pvt_traj();
            has_goal_ = false;
        }
    }
    set_result(-100, "TM_ROS: Goal cancelled by client");
    gh.setCanceled(result_);
}


////////////////////////////////

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tm_driver");

    /*for (int i = 0; i < argc; ++i){
        print_info("arg[%d]: %s", i, argv[i]);
    }*/

    std::string host;
    if (!ros::param::get("~robot_ip_address", host)) {
        if (argc > 1) {
            host = argv[1];
        }
        else return 1;
    }
    print_info("TM_ROS: robot_ip:=%s", host.c_str());

    TmRosNode robot(host);

    ros::AsyncSpinner spinner(2);
    spinner.start();

    ros::waitForShutdown();

    //robot.halt();

    printf("TM_ROS: shutdown\n");
    return 0;
}
