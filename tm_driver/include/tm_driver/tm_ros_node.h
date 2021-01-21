
#include "tm_driver/tm_print.h"
#include "tm_driver/tm_driver.h"

#include <ros/ros.h>

#include <actionlib/server/action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
//#include <visualization_msgs/InteractiveMarkerUpdate.h>

//#include <boost/chrono/chrono.hpp>
//#include <boost/thread/thread.hpp>
//#include <boost/thread/mutex.hpp>
//#include <boost/thread/condition_variable.hpp>

#include "tm_driver/tm_pose_conversion.h"

#include "tm_msgs/FeedbackState.h"
#include "tm_msgs/SvrResponse.h"
#include "tm_msgs/SctResponse.h"
#include "tm_msgs/StaResponse.h"
#include "tm_msgs/ConnectTM.h"
#include "tm_msgs/WriteItem.h"
#include "tm_msgs/AskItem.h"
#include "tm_msgs/SendScript.h"
#include "tm_msgs/SetEvent.h"
#include "tm_msgs/SetIO.h"
//#include "tm_msgs/SetPayload"
#include "tm_msgs/SetPositions.h"
#include "tm_msgs/AskSta.h"


class TmRosNode {
protected:
    std::condition_variable svr_cv_;
    std::condition_variable sct_cv_;
    TmDriver iface_;

    ros::NodeHandle nh_;

    ////////////////////////////////
    // Action
    ////////////////////////////////

    // joint_trajectory_action
    actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction> as_;
    actionlib::ServerGoalHandle<control_msgs::FollowJointTrajectoryAction> gh_;
    control_msgs::FollowJointTrajectoryResult result_;
    bool has_goal_;

    ////////////////////////////////
    // Param.
    ////////////////////////////////

    std::vector<std::string> joint_names_;
    std::string base_frame_name_;
    std::string tool_frame_name_;

    ////////////////////////////////
    // Topic
    ////////////////////////////////

    struct PubMsg {
        ros::Publisher fbs_pub;
        ros::Publisher joint_pub;
        ros::Publisher tool_pose_pub;
        ros::Publisher svr_pub;

        tm_msgs::FeedbackState fbs_msg;
        sensor_msgs::JointState joint_msg;
        geometry_msgs::PoseStamped tool_pose_msg;

        //tf::Transform transform;
        tf::TransformBroadcaster tfbc;

        tm_msgs::SvrResponse svr_msg;
    } pm_;

    struct SctMsg {
        ros::Publisher sct_pub;
        ros::Publisher sta_pub;

        tm_msgs::SctResponse sct_msg;
        tm_msgs::StaResponse sta_msg;
    } sm_;

    bool svr_updated_;
    boost::mutex svr_mtx_;
    boost::condition_variable svr_cond_;

    int pub_reconnect_timeout_ms_;
    int pub_reconnect_timeval_ms_;
    boost::thread pub_thread_;

    bool sta_updated_;
    boost::mutex sta_mtx_;
    boost::condition_variable sta_cond_;

    int sct_reconnect_timeout_ms_;
    int sct_reconnect_timeval_ms_;
    boost::thread sct_thread_;

    ////////////////////////////////
    // Service for connection
    ////////////////////////////////

    ros::ServiceServer connect_srv_;

    ////////////////////////////////
    // Service
    ////////////////////////////////

    ros::ServiceServer write_item_srv_;
    ros::ServiceServer ask_item_srv_;

    ros::ServiceServer send_script_srv_;

    ros::ServiceServer set_event_srv_;
    ros::ServiceServer set_io_srv_;

    ros::ServiceServer set_positions_srv_;

    ros::ServiceServer ask_sta_srv_;

    ////////////////////////////////
    // Init.
    ////////////////////////////////
public:
    TmRosNode(const std::string &host);
    ~TmRosNode();
    void halt();

private:
    ////////////////////////////////
    // Action
    ////////////////////////////////

    // helper function

    bool has_points(const trajectory_msgs::JointTrajectory &traj);
    bool has_limited_velocities(const trajectory_msgs::JointTrajectory &traj);
    bool is_traj_finite(const trajectory_msgs::JointTrajectory &traj);
    void reorder_traj_joints(trajectory_msgs::JointTrajectory &traj);
    bool is_start_positions_match(const trajectory_msgs::JointTrajectory &traj, double eps);
    void set_result(int32_t err_code, const std::string &err_str);
    //void print_traj(const trajectory_msgs::JointTrajectory &traj);

    void set_pvt_traj(TmPvtTraj &pvts, const trajectory_msgs::JointTrajectory &traj);
    void traj_action(TmPvtTraj pvts);

    // action function

    void goalCB(actionlib::ServerGoalHandle<control_msgs::FollowJointTrajectoryAction> gh);
    void cancelCB(actionlib::ServerGoalHandle<control_msgs::FollowJointTrajectoryAction> gh);

    ////////////////////////////////
    // Topic
    ////////////////////////////////

    void publish_fbs();
    void publish_svr();
    bool publish_func();
    void publisher();

    void sct_msg();
    void sta_msg();
    bool sct_func();
    void sct_responsor();

    ////////////////////////////////
    // Service
    ////////////////////////////////

    bool connect_tm(tm_msgs::ConnectTMRequest &req, tm_msgs::ConnectTMResponse &res);

    bool write_item(tm_msgs::WriteItemRequest &req, tm_msgs::WriteItemResponse &res);
    bool ask_item(tm_msgs::AskItemRequest &req, tm_msgs::AskItemResponse &res);

    bool send_script(tm_msgs::SendScriptRequest &req, tm_msgs::SendScriptResponse &res);

    bool set_event(tm_msgs::SetEventRequest &req, tm_msgs::SetEventResponse &res);
    bool set_io(tm_msgs::SetIORequest &req, tm_msgs::SetIOResponse &res);

    bool set_positions(tm_msgs::SetPositionsRequest &req, tm_msgs::SetPositionsResponse &res);

    bool ask_sta(tm_msgs::AskStaRequest &req, tm_msgs::AskStaResponse &res);
};
