// a pick ang place demo, initialized in 211124
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <vector>
#include <std_msgs/Int8.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Pose.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <tf/transform_listener.h>
#include <iostream>
#include <math.h>
#include <ros/ros.h>

#include <ur_dashboard_msgs/Load.h>
#include <ur_dashboard_msgs/RobotMode.h> 
#include <ur_dashboard_msgs/GetSafetyMode.h>
#include <controller_manager_msgs/SwitchController.h>

#include "mission_mode/mode_switch.h"
#include "ur5e_pid_control/PIDPoseTwist.h"

#include <time.h>
#include <stdio.h>
#include <unistd.h>
#include <fstream>

class Manipulator
{
public:
    Manipulator();
    void obj_callback(const geometry_msgs::PoseStamped& obj_visual_pose);
    void slam_callback(const nav_msgs::Odometry& slam_odom);
    void state_arm_callback(const ur5e_pid_control::PIDPoseTwistConstPtr msg);
    void global_command_callback(const geometry_msgs::PoseStamped& desired_ee_global_pose);
 


    void open_gripper();
    void close_gripper();

    void pick(moveit::planning_interface::MoveGroupInterface& arm_group);
    void tracking();
    void stablize();

    bool plan_mode(ros::ServiceClient& controller_switch_);
    bool custom_pid_mode(ros::ServiceClient& controller_switch_);
    bool custom_adm_mode(ros::ServiceClient& controller_switch_);


    void visual_servo();
    void memory_servo();
    void preset_servo();


public:
    ros::NodeHandle n_; 
    ros::Subscriber sub_pose_;
    ros::Subscriber sub_slam_odom_;
    ros::Subscriber sub_arm_state_;
    ros::Subscriber sub_global_command_;
    ros::Publisher equilibrium_info_pub_;
    ros::Publisher pub_grip_;
    ros::ServiceClient controller_switch_;

    int servo_flag;

    float grip_force;
    float grip_position;

    int maxtries;   // max attempts
    double jump_threshold;
    double eef_step;

    geometry_msgs::Twist grip_msg;
    geometry_msgs::PoseStamped ee_pose_msg;

    tf::TransformListener listener;
    // tf::StampedTransform ee_transform;
    // tf::StampedTransform map_arm_transform;

    double visual_last_stamp_;
    int visual_current_seq_;
    int visual_last_seq_;
    float obj_visual_yaw_;
    
    // controller switch related
    int controller_flag_;

    Eigen::Vector3f obj_visual_translation_;
    Eigen::Quaternionf obj_visual_direct_orientation_;

    Eigen::Vector3f base_map_translation_;
    Eigen::Quaternionf base_map_orientation_;

    Eigen::Vector3f arm_real_position_;
    Eigen::Quaternionf arm_real_orientation_;

    Eigen::Vector3f desired_ee_map_position_;
    Eigen::Quaternionf desired_ee_map_orientation_;

    // static transformation which wont be updated
    Eigen::Vector3f static_lidar_arm_translation;
    Eigen::Quaternionf static_lidar_arm_orientation;
};
