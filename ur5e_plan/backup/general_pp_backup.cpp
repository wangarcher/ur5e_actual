// a simple planning demo, initialized in 210813
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
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <iostream>
#include <math.h>
#include <ros/ros.h>


class Manipulator
{
public:
    Manipulator()
    {

        servo_flag = 0;

        grip_force = 1.0;
        grip_position = 0;


        kp = 0.001;
        ki = 0.01;
        kd = 0.01;

        maxtries = 100;   // max attempts
        jump_threshold = 0.0;
        eef_step = 0.002;

        // the move flag remains here, to be replace by the base planner and controller
        // see if the ego-planner offers the interface like that?
        // sub_move_ = n_.subscribe("/move_flag", 1000, &Manipulator::flag_callback, this);

        // we now set the object pose manually
        sub_pose_ = n_.subscribe("/aruco_ros/pose", 1000, &Manipulator::obj_callback, this);
        
        // disable temporaily
        // sub_loc_ = n_.subscribe("/base_location", 1000, &Manipulator::base_callback, this);

        // for remote control, not useful for this project, now deprecated
        // romote_controller_sub_ = n_.subscribe("/arm_control", 5, &Manipulator::remote_control_info_callback, this);

        // command of the gripper, for the real robot DH-gripper
        pub_grip_ = n_.advertise<geometry_msgs::Twist>("/grip_control", 1000);

        servo_info_pub_ = n_.advertise<geometry_msgs::TwistStamped>("/servo_server/delta_twist_cmds",5);

        // arm mode switch, not useful, to be deprecated
        // switch_service_ = n_.advertiseService("/arm_mode_switch",arm_mode_switch_handle);

        // controller switch, by Wang ju, his contribution shall be 
        controller_switch_ = n_.serviceClient<controller_manager_msgs::SwitchController>("/controller_manager/switch_controller");
    }

public:
    void flag_callback(const std_msgs::Int8& flag);
    void obj_callback(const geometry_msgs::TwistStamped& v_pose);
    // void base_callback(const geometry_msgs::TwistStamped& b_pose);
    // void remote_control_info_callback(const geometry_msgs::Twist::ConstPtr& msg);

    void open_gripper(trajectory_msgs::JointTrajectory& posture);
    void close_gripper(trajectory_msgs::JointTrajectory& posture);

    void pick(moveit::planning_interface::MoveGroupInterface& arm_group);
    void place(moveit::planning_interface::MoveGroupInterface& arm_group);

    void add_collsions(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface);

    bool servo_mode(ros::ServiceClient& controller_switch);
    bool plan_mode(ros::ServiceClient& controller_switch);
 
    float pid(float obj_value, float ee_value);


public:
    ros::NodeHandle n_; 
    ros::Subscriber sub_pose_;
    ros::Publisher servo_info_pub_;
    ros::Publisher pub_grip_;
    ros::ServiceClient controller_switch_;

    int servo_flag;

    float grip_force;
    float grip_position;

    float kp;
    float ki;
    float kd;

    int maxtries;   // max attempts
    double jump_threshold;
    double eef_step;

    geometry_msgs::Twist grip_msg;
    geometry_msgs::TwistStamped servo_msg;
    geometry_msgs::PoseStamped visual_msg;
    geometry_msgs::PoseStamped base_msg;
    geometry_msgs::PoseStamped obj_msg;

    tf::TransformListener listener;
    tf::StampedTransform ee_transform;


};


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
    // the way points of the pre pick up point
    std::vector<geometry_msgs::Pose> waypoints;
    geometry_msgs::Pose pose; // better if we can have a geometry_msgs directly
    pose.position.x = pre_x;
    pose.position.y = pre_y;	
    pose.position.z = pre_z;
    pose.orientation.x = pre_q.x();
    pose.orientation.y = pre_q.y();
    pose.orientation.z = pre_q.z();
    pose.orientation.w = pre_q.w();
    waypoints.push_back(pose);

    moveit_msgs::RobotTrajectory trajectory;
    double fraction = 0.0;
    int attempts = 0;     // attempts done

    while(fraction < 1.0 && attempts < maxtries)
    {
        fraction = arm.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
        attempts++;
        if(attempts % 10 == 0) ROS_INFO("Still trying after %d attempts...", attempts);
    }
    if(fraction == 1)
    {   
        ROS_INFO("Path computed successfully. Moving the arm.");
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        plan.trajectory_ = trajectory;
        arm.execute(plan);   
        sleep(1);
    }
    else
    {
        ROS_INFO("Path planning failed with only %0.6f success after %d attempts.", fraction, maxtries);
    }
    servo_mode(controller_switch_);
    servo_flag = 1;
}

void Manipulator::place(moveit::planning_interface::MoveGroupInterface& arm_group)
{
    plan_mode(controller_switch_);

    std::vector<geometry_msgs::Pose> waypoints;
    geometry_msgs::Pose pose2; // better if we can have a geometry_msgs directly
    pose2.position.x = pre_x;
    pose2.position.y = pre_y;	
    pose2.position.z = pre_z;
    pose2.orientation.x = pre_q.x();
    pose2.orientation.y = pre_q.y();
    pose2.orientation.z = pre_q.z();
    pose2.orientation.w = pre_q.w();
    waypoints.push_back(pose2);

    moveit_msgs::RobotTrajectory trajectory;
    double fraction = 0.0;
    int attempts = 0;     // attempts done

    while(fraction < 1.0 && attempts < maxtries)
    {
        fraction = arm.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
        attempts++;
        if(attempts % 10 == 0) ROS_INFO("Still trying after %d attempts...", attempts);
    }
    if(fraction == 1)
    {   
        ROS_INFO("Path computed successfully. Moving the arm.");
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        plan.trajectory_ = trajectory;
        arm.execute(plan);   
        sleep(1);
    }
    else
    {
        ROS_INFO("Path planning failed with only %0.6f success after %d attempts.", fraction, maxtries);
    }
    memory_servo(arm_group, base_msg, obj_msg);

}

bool Manipulator::servo_mode(ros::ServiceClient& controller_switch_)
{
    controller_manager_msgs::SwitchController switcher;

    switcher.request.stop_controllers.push_back("scaled_pos_joint_traj_controller");
    switcher.request.stop_controllers.push_back("pos_joint_traj_controller");
    switcher.request.start_controllers.push_back("joint_group_vel_controller");

    switcher.request.strictness = 0;
    switcher.request.start_asap = false;
    switcher.request.timeout = 0.;

    controller_switch_.call(switcher);

    return switcher.response.ok;
}

bool Manipulator::plan_mode(ros::ServiceClient& controller_switch_)
{
    controller_manager_msgs::SwitchController switcher;

    switcher.request.stop_controllers.push_back("joint_group_vel_controller");
    switcher.request.start_controllers.push_back("scaled_pos_joint_traj_controller");
    switcher.request.start_controllers.push_back("pos_joint_traj_controller");

    switcher.request.strictness = 0;
    switcher.request.start_asap = false;
    switcher.request.timeout = 0.;

    controller_switch_.call(switcher);

    return switcher.response.ok;
}

void Manipulator::obj_callback(const geometry_msgs::PoseStamped& v_pose)
{
    try
    {
        listener.lookupTransform("/base_link","/tool0", ros::Time(0), ee_transform);
    }
    catch(tf::TransformException &ex)
    {
        ROS_ERROR("%s", ex.what());
        ros::Duration(1.0).sleep();
        continue;
    }
    float ee_l_x = ee_transform.getOrigin().x();
    float ee_l_y = ee_transform.getOrigin().y();
    float ee_l_z = ee_transform.getOrigin().z();
    // float ee_roll;
    // float ee_pitch;
    // float ee_yaw;
    // tf::Matrix3x3(ee_transform.getRotation()).getEulerYPR(ee_yaw, ee_pitch, ee_roll);
    Eigen::Vector3f ee_t(ee_l_x, ee_l_y, ee_l_z);
    Eigen::Vector3f obj_v_t(v_pose.pose.position.x, v_pose.pose.position.y, v_pose.pose.position.z);
    Eigen::Quaternionf ee_b_q(ee_transform.getRotation().w(), 
                         ee_transform.getRotation().x(),
                         ee_transform.getRotation().y(),
                         ee_transform.getRotation().z());
    Eigen::Vector3f obj_b_t;
    obj_b_t = ee_b_q * obj_v_t + ee_t;
    if(servo_flag == 1)
    {
        float linear_x = pid(obj_b_t(0), ee_l_x);
        float linear_y = pid(obj_b_t(1), ee_l_y);
        float linear_z = pid(obj_b_t(2), ee_l_z);
        // float angular_x = pid(obj_a_x, ee_roll);
        // float angular_y = pid(obj_a_y, ee_pitch);
        // float angular_z = pid(obj_a_z, ee_yaw);
	    servo_msg.twist.linear.x = linear_x;
	    servo_msg.twist.linear.y = linear_y;
	    servo_msg.twist.linear.z = linear_z;
        servo_msg.twist.angular.x = 0;// set as 0 temporaily
        servo_msg.twist.angular.y = 0;
	    servo_msg.twist.angular.z = 0;
        servo_info_pub_.publish(servo_msg);
    }
}

float Manipulator::pid(float obj_value, float ee_value)
{
    float error_last = 0;
    float error_next = 0;
    float error = obj_value - ee_value;
    float increment_value =  kp * (error - error_next) + ki * error + kd * (error - 2 * error_next + error_last);
    error_last = error_next;
    error_next = error;
    return increment_value
}

void Manipulator::add_collisions(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface)
{
  planning_scene_interface.applyCollisionObjects(collision_objects);
}

int main(int argc, char **argv)
{
    
    ros::init(argc, argv, "project_plan");
    Manipulator manipulator;
    ros::AsyncSpinner spinner(2);
	spinner.start();


    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    manipulator.addCollisionObjects(planning_scene_interface);

    // arm group initialization
    moveit::planning_interface::MoveGroupInterface arm("manipulator");
    arm.setPoseReferenceFrame("base_link");
    arm.allowReplanning(true);         // Evangelion 3.33: You Can(Not) Redo.
    arm.setGoalJointTolerance(0.01);
    arm.setGoalPositionTolerance(0.01); // set position torlerance
    arm.setGoalOrientationTolerance(0.01);  // set orientation torlerance
    arm.setMaxAccelerationScalingFactor(0.7); // set max accelerations
    arm.setMaxVelocityScalingFactor(0.7);

    ros::WallDuration(1.0).sleep();

    manipulator.pick(arm);

    manipulator.place(arm);



 
    ros::waitForShutdown();

    return 0;
}