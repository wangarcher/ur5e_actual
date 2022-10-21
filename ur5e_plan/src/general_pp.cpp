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
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/transform_listener.h>

#include <time.h>
#include <stdio.h>
#include <unistd.h>
#include <fstream>
#include <math.h>
#include <ros/ros.h>
#include <iostream>

#include <ur_dashboard_msgs/Load.h>
#include <ur_dashboard_msgs/RobotMode.h> 
#include <ur_dashboard_msgs/GetSafetyMode.h>

#include "mission_mode/mode_switch.h"
#include "general_pp.h"

Manipulator::Manipulator()
{
    visual_last_seq_ = -1;
    visual_last_stamp_ = 0.0;
    arm_real_position_.setZero();

    obj_visual_yaw_ = 0.0;

    grip_force = 1.0;
    grip_position = 0;

    maxtries = 100;   // max attempts
    jump_threshold = 0.0;
    eef_step = 0.002;


    controller_flag_ = 0; // set as planning mode for pereferenece

    static_lidar_arm_translation << 0.425, 0.0, -0.75;
    static_lidar_arm_orientation = Eigen::AngleAxisf(1.57, Eigen::Vector3f::UnitZ()) *
                                    Eigen::AngleAxisf(0.0, Eigen::Vector3f::UnitY()) *
                                    Eigen::AngleAxisf(0.0, Eigen::Vector3f::UnitX());

    // sub_pose_ = n_.subscribe("/aruco_single/pose", 10, &Manipulator::obj_callback, this);

    sub_slam_odom_ = n_.subscribe("/lio_sam/mapping/odometry", 1, &Manipulator::slam_callback, this, ros::TransportHints().tcpNoDelay());

    sub_global_command_ = n_.subscribe("/global_ee_command", 1, &Manipulator::global_command_callback, this, ros::TransportHints().tcpNoDelay());
    // sub_arm_state_ = n_.subscribe("/ur5e_cartesian_velocity_controller_pid/ee_state", 10,
    //                              &Manipulator::state_arm_callback, this,
    //                              ros::TransportHints().reliable().tcpNoDelay());

    // command of the gripper, for the real robot DH-gripper
    pub_grip_ = n_.advertise<geometry_msgs::Twist>("/grip_control", 1); 

    equilibrium_info_pub_ = n_.advertise<geometry_msgs::PoseStamped>("/now_equilibrium", 1);

    // controller switch, by Wang ju, his contribution shall be 
    controller_switch_ = n_.serviceClient<controller_manager_msgs::SwitchController>("/controller_manager/switch_controller");
}

void Manipulator::open_gripper()
{
    grip_position = 1.0;
    grip_msg.linear.x = grip_force;
    grip_msg.linear.y = grip_position;
    pub_grip_.publish(grip_msg);
}

void Manipulator::close_gripper()
{
    grip_position = 0.0;
    grip_msg.linear.x = grip_force;
    grip_msg.linear.y = grip_position;
    pub_grip_.publish(grip_msg);
}

void Manipulator::pick(moveit::planning_interface::MoveGroupInterface& arm_group)
{
    plan_mode(controller_switch_);
    std::vector<geometry_msgs::Pose> waypoints;
    geometry_msgs::Pose pose; // better if we can have a geometry_msgs directly
    pose.position.x = -0.14;
    pose.position.y = 0.30;	
    pose.position.z = 0.45;
    pose.orientation.x = -0.732592;
    pose.orientation.y = 0.00365944;
    pose.orientation.z = 0.0034352;
    pose.orientation.w = 0.680649;
    waypoints.push_back(pose);

    moveit_msgs::RobotTrajectory trajectory;
    double fraction = 0.0;
    int attempts = 0;     // attempts done

    while(fraction < 1.0 && attempts < maxtries)
    {
        fraction = arm_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
        attempts++;
        if(attempts % 10 == 0) ROS_INFO("Still trying after %d attempts...", attempts);
    }
    if(fraction == 1)
    {   
        ROS_INFO("Path computed successfully. Moving the arm.");
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        plan.trajectory_ = trajectory;
        arm_group.execute(plan);   
        sleep(1);
    }
    else
    {
        ROS_INFO("Path planning failed with only %0.6f success after %d attempts.", fraction, maxtries);
    }
    custom_pid_mode(controller_switch_);
    visual_servo();
}

void Manipulator::tracking()
{
    custom_pid_mode(controller_switch_);
    memory_servo();
}

void Manipulator::stablize()
{
    custom_pid_mode(controller_switch_);
    preset_servo();
}

bool Manipulator::plan_mode(ros::ServiceClient& controller_switch_)
{
    controller_manager_msgs::SwitchController switcher;

    switcher.request.stop_controllers.push_back("ur5e_cartesian_velocity_controller_adm");
    switcher.request.stop_controllers.push_back("ur5e_cartesian_velocity_controller_pid");
    switcher.request.stop_controllers.push_back("joint_group_vel_controller");
    switcher.request.start_controllers.push_back("scaled_pos_joint_traj_controller");

    switcher.request.strictness = 0;
    switcher.request.start_asap = false;
    switcher.request.timeout = 0.;
    controller_switch_.call(switcher);

    return switcher.response.ok;
}

bool Manipulator::custom_pid_mode(ros::ServiceClient& controller_switch_)
{
    controller_manager_msgs::SwitchController switcher;

    switcher.request.stop_controllers.push_back("ur5e_cartesian_velocity_controller_adm");
    switcher.request.stop_controllers.push_back("scaled_pos_joint_traj_controller");
    switcher.request.stop_controllers.push_back("pos_joint_traj_controller");
    switcher.request.stop_controllers.push_back("joint_group_vel_controller");
    switcher.request.start_controllers.push_back("ur5e_cartesian_velocity_controller_pid");

    switcher.request.strictness = 0;
    switcher.request.start_asap = false;
    switcher.request.timeout = 0.;
    controller_switch_.call(switcher);

    return switcher.response.ok;
}

bool Manipulator::custom_adm_mode(ros::ServiceClient& controller_switch_)
{
    controller_manager_msgs::SwitchController switcher;

    switcher.request.stop_controllers.push_back("ur5e_cartesian_velocity_controller_pid");
    switcher.request.stop_controllers.push_back("scaled_pos_joint_traj_controller");
    switcher.request.stop_controllers.push_back("pos_joint_traj_controller");
    switcher.request.stop_controllers.push_back("joint_group_vel_controller");
    switcher.request.start_controllers.push_back("ur5e_cartesian_velocity_controller_adm");

    switcher.request.strictness = 0;
    switcher.request.start_asap = false;
    switcher.request.timeout = 0.;
    controller_switch_.call(switcher);

    return switcher.response.ok;
}

void Manipulator::visual_servo()
{
    while(n_.ok() && controller_flag_ == 1)
    {
        Eigen::Vector3f obj_base_translation;
        Eigen::Vector3f obj_base_last_translation;
        Eigen::Quaternionf obj_base_orientation;
        Eigen::Quaternionf obj_base_last_orientation;

        if(visual_current_seq_ != 0)
        {
            if(visual_last_seq_ == visual_current_seq_)
            {
                obj_base_translation = obj_base_last_translation;
                obj_base_orientation = obj_base_last_orientation;
            }
            else
            {
                obj_base_translation = arm_real_orientation_ * obj_visual_translation_ + arm_real_position_;

                float obj_visual_roll = atan(obj_visual_translation_(1)/obj_visual_translation_(2));
                float obj_visual_pitch = atan(obj_visual_translation_(0)/obj_visual_translation_(2));
                obj_visual_direct_orientation_ = Eigen::AngleAxisf(obj_visual_yaw_, Eigen::Vector3f::UnitZ()) *
                                                Eigen::AngleAxisf(obj_visual_pitch, Eigen::Vector3f::UnitY()) *
                                                Eigen::AngleAxisf(obj_visual_roll, Eigen::Vector3f::UnitX());
                obj_base_orientation = obj_visual_direct_orientation_.inverse() * arm_real_orientation_;

                obj_base_last_orientation = obj_base_orientation;
                obj_base_last_translation = obj_base_translation;
                visual_last_seq_ = visual_current_seq_;
            }
        }

        // std::cout << "the desired ee pose: "<< desired_ee_arm_position.transpose() << std::endl;
        ee_pose_msg.pose.position.x = obj_base_translation(0);
        ee_pose_msg.pose.position.y = obj_base_translation(1);
        ee_pose_msg.pose.position.z = obj_base_translation(2);
        ee_pose_msg.pose.orientation.x = obj_base_orientation.x();
        ee_pose_msg.pose.orientation.y = obj_base_orientation.y();
        ee_pose_msg.pose.orientation.z = obj_base_orientation.z();
        ee_pose_msg.pose.orientation.w = obj_base_orientation.w();
        ee_pose_msg.header.frame_id = "pid";
        equilibrium_info_pub_.publish(ee_pose_msg);
    }
}

void Manipulator::memory_servo()
{
    Eigen::Vector3f arm_map_translation;
    Eigen::Quaternionf arm_map_orientation;


    while(n_.ok() && controller_flag_ == 1)
    {
        arm_map_orientation = base_map_orientation_ * static_lidar_arm_orientation;
        arm_map_translation = base_map_orientation_.toRotationMatrix() * static_lidar_arm_translation + base_map_translation_;
        Eigen::Quaternionf desired_ee_arm_orientation = arm_map_orientation.inverse() * desired_ee_map_orientation_;
        Eigen::Vector3f desired_ee_arm_position = arm_map_orientation.toRotationMatrix().inverse() * (desired_ee_map_position_ - arm_map_translation);
        // std::cout << "desired_ee_arm_position: "<< desired_ee_arm_position.transpose() << std::endl;

        ee_pose_msg.pose.position.x = desired_ee_arm_position(0);
        ee_pose_msg.pose.position.y = desired_ee_arm_position(1);
        ee_pose_msg.pose.position.z = desired_ee_arm_position(2);
        ee_pose_msg.pose.orientation.x = desired_ee_arm_orientation.x();
        ee_pose_msg.pose.orientation.y = desired_ee_arm_orientation.y();
        ee_pose_msg.pose.orientation.z = desired_ee_arm_orientation.z();
        ee_pose_msg.pose.orientation.w = desired_ee_arm_orientation.w();
        ee_pose_msg.header.frame_id = "pid";
        equilibrium_info_pub_.publish(ee_pose_msg);
        ros::Duration(0.001).sleep();
    }
}

void Manipulator::preset_servo()
{
    Eigen::Vector3f arm_map_translation;
    Eigen::Quaternionf arm_map_orientation;

    base_map_translation_ << 0.0,
                             0.0,
                             0.0;
    base_map_orientation_.coeffs() <<  0.0,
                                       0.0,
                                       0.0,
                                       1.0;
    desired_ee_map_position_ << 1.0,
                                0.0,
                                -0.15;
    desired_ee_map_orientation_ = Eigen::AngleAxisf(0.0, Eigen::Vector3f::UnitZ()) *
                                  Eigen::AngleAxisf(1.57, Eigen::Vector3f::UnitY()) *
                                  Eigen::AngleAxisf(0, Eigen::Vector3f::UnitX());

    while(n_.ok())
    {
        arm_map_orientation = base_map_orientation_ * static_lidar_arm_orientation;
        arm_map_translation = base_map_orientation_.toRotationMatrix() * static_lidar_arm_translation + base_map_translation_;
        Eigen::Quaternionf desired_ee_arm_orientation = arm_map_orientation.inverse() * desired_ee_map_orientation_;
        Eigen::Vector3f desired_ee_arm_position = arm_map_orientation.toRotationMatrix().inverse() * (desired_ee_map_position_ - arm_map_translation);
        std::cout << "desired_ee_arm_position: "<< desired_ee_arm_position.transpose() << std::endl;
        std::cout << "desired_ee_arm_orientation: " << desired_ee_arm_orientation.w() << ", "  
                                                  << desired_ee_arm_orientation.x() << ", "
                                                  << desired_ee_arm_orientation.y() << ", " 
                                                  << desired_ee_arm_orientation.z() << std::endl; 
        ee_pose_msg.pose.position.x = desired_ee_arm_position(0);
        ee_pose_msg.pose.position.y = desired_ee_arm_position(1);
        ee_pose_msg.pose.position.z = desired_ee_arm_position(2);
        ee_pose_msg.pose.orientation.x = desired_ee_arm_orientation.x();
        ee_pose_msg.pose.orientation.y = desired_ee_arm_orientation.y();
        ee_pose_msg.pose.orientation.z = desired_ee_arm_orientation.z();
        ee_pose_msg.pose.orientation.w = desired_ee_arm_orientation.w();
        ee_pose_msg.header.frame_id = "pid";
        equilibrium_info_pub_.publish(ee_pose_msg);
        ros::Duration(0.001).sleep();
    }
}

// hold one second 
// void Manipulator::preset_servo()
// {
//     int preset_buffer_count = 0;
//     while(n_.ok() && controller_flag_ == 2 && preset_buffer_count < 100)
//     {
//         ee_pose_msg.header.frame_id = "adm";
//         equilibrium_info_pub_.publish(ee_pose_msg);
//         ros::Duration(0.1).sleep();
//         preset_buffer_count++;
//     }
// }

void Manipulator::obj_callback(const geometry_msgs::PoseStamped& obj_visual_pose)
{
    visual_last_stamp_ = ros::Time::now().toSec();
    visual_current_seq_ = obj_visual_pose.header.seq;
    obj_visual_translation_ << obj_visual_pose.pose.position.x,
                               obj_visual_pose.pose.position.y,
                               obj_visual_pose.pose.position.z;
    Eigen::Quaternionf mid_term_1(obj_visual_pose.pose.orientation.w, 
                                  obj_visual_pose.pose.orientation.x,
                                  obj_visual_pose.pose.orientation.y,
                                  obj_visual_pose.pose.orientation.z);
    Eigen::Vector3f obj_visual_euler = mid_term_1.matrix().eulerAngles(2, 1, 0);
    obj_visual_yaw_ = obj_visual_euler(0);
}

void Manipulator::slam_callback(const nav_msgs::Odometry& slam_odom)
{
    base_map_translation_ << slam_odom.pose.pose.position.x,
                             slam_odom.pose.pose.position.y,
                             slam_odom.pose.pose.position.z;
    base_map_orientation_.coeffs() <<  slam_odom.pose.pose.orientation.x,
                                       slam_odom.pose.pose.orientation.y,
                                       slam_odom.pose.pose.orientation.z,
                                       slam_odom.pose.pose.orientation.w;
}

void Manipulator::state_arm_callback(const ur5e_pid_control::PIDPoseTwistConstPtr msg) {
    arm_real_position_ << msg->pose.position.x,
                          msg->pose.position.y,
                          msg->pose.position.z;
    arm_real_orientation_.coeffs() << msg->pose.orientation.x,
                                      msg->pose.orientation.y,
                                      msg->pose.orientation.z,
                                      msg->pose.orientation.w;
}

void Manipulator::global_command_callback(const geometry_msgs::PoseStamped& desired_ee_global_pose)
{
    desired_ee_map_position_ << desired_ee_global_pose.pose.position.x,
                                desired_ee_global_pose.pose.position.y,
                                desired_ee_global_pose.pose.position.z;
    desired_ee_map_orientation_.coeffs() << desired_ee_global_pose.pose.orientation.x,
                                            desired_ee_global_pose.pose.orientation.y,
                                            desired_ee_global_pose.pose.orientation.z,
                                            desired_ee_global_pose.pose.orientation.w;
    if(desired_ee_global_pose.header.frame_id == "pid") controller_flag_ = 1;
    if(desired_ee_global_pose.header.frame_id == "adm") controller_flag_ = 2;
}