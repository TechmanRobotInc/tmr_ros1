#include "tm_driver/tm_ros_node.h"

////////////////////////////////
// robot error handling
////////////////////////////////

void TmRosNode::robot_error_proc(int &count)
{
}

////////////////////////////////
// Topic
////////////////////////////////

void TmRosNode::publish_msg(PubMsg &pm)
{
    // Publish feedback state
    pm.fbs_msg.header.stamp = ros::Time::now();

    pm.fbs_msg.is_svr_connected = iface_.svr.is_connected();
    pm.fbs_msg.is_sct_connected = iface_.sct.is_connected();

    pm.fbs_msg.joint_pos = iface_.state.joint_angle();
    pm.fbs_msg.joint_vel = iface_.state.joint_speed();
    pm.fbs_msg.joint_tor = iface_.state.joint_torque();
    pm.fbs_msg.tool_pose = iface_.state.tool_pose();
    pm.fbs_msg.tcp_speed = iface_.state.tcp_speed_vec();
    pm.fbs_msg.robot_link = iface_.state.is_linked();
    pm.fbs_msg.robot_error = iface_.state.has_error();
    pm.fbs_msg.project_run = iface_.state.is_project_running();
    pm.fbs_msg.project_pause = iface_.state.is_project_paused();
    pm.fbs_msg.safetyguard_a = iface_.state.is_safeguard_A();
    pm.fbs_msg.e_stop = iface_.state.is_EStop();
    pm.fbs_msg.error_code = iface_.state.error_code();
    pm.fbs_msg.error_content = iface_.state.error_content();
    pm.fbs_msg.cb_digital_input = iface_.state.ctrller_DI();
    pm.fbs_msg.cb_analog_input = iface_.state.ctrller_AI();
    pm.fbs_msg.ee_digital_input = iface_.state.ee_DI();
    pm.fbs_msg.ee_analog_input = iface_.state.ee_AI();
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

void TmRosNode::publisher()
{
    print_info("TM_ROS: publisher thread begin");

    PubMsg pm;
    pm.fbs_pub = nh_.advertise<tm_msgs::FeedbackState>("feedback_states", 1);
    pm.joint_pub = nh_.advertise<sensor_msgs::JointState>("joint_states", 1);
    pm.tool_pose_pub = nh_.advertise<geometry_msgs::PoseStamped>("tool_pose", 1);

    pm.joint_msg.name = joint_names_;
    pm.joint_msg.position.assign(joint_names_.size(), 0.0);
    pm.joint_msg.velocity.assign(joint_names_.size(), 0.0);
    pm.joint_msg.effort.assign(joint_names_.size(), 0.0);

    while (ros::ok()) {
        int count = 0;
        bool reconnect = false;
        while (ros::ok() && iface_.svr.is_connected() && !reconnect){
            auto rc = iface_.svr.tmsvr_function();
            /*if (rc == TmCommRC::OK) {
                robot_error_proc(count);
            }*/
            switch (rc) {
            case TmCommRC::OK:
                publish_msg(pm);
                break;
            case TmCommRC::NOTREADY:
            case TmCommRC::NOTCONNECT:
            case TmCommRC::ERR:
                print_info("TM_ROS: (TM_SVR) rc=%d", int(rc));
                reconnect = true;
                break;
            default: break;
            }
        }
        iface_.svr.Close();
        print_info("TM_ROS: (TM_SVR) reconnect in ");
        int cnt = 5;
        while (ros::ok() && cnt > 0) {
            print_info("%d sec...", cnt);
            boost::this_thread::sleep_for(boost::chrono::seconds(1));
            --cnt;
        }
        if (ros::ok()) {
            print_info("TM_ROS: (TM_SVR) connect...");
            iface_.svr.Connect(1000);
        }
    }
    iface_.svr.Close();
    printf("TM_ROS: publisher thread end\n");
}
