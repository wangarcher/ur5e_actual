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
        move_flag = 0;
        view_flag = 0;
        grip_force = 1.0;
        grip_position = 0;
        simulation = true;

        // the move flag remains here, to be replace by the base planner and controller
        // see if the ego-planner offers the interface like that?
        sub_move_ = n_.subscribe("/move_flag", 1000, &Manipulator::flag_callback, this);

        // we now set the object pose manually
        sub_pose_ = n_.subscribe("/obj_pose", 1000, &Manipulator::obj_callback, this);

        // command of the gripper, for the real robot DH-gripper
        pub_grip_ = n_.advertise<geometry_msgs::Twist>("/grip_control", 1000);

    }

public:
    void flag_callback(const std_msgs::Int8& flag);
    void obj_callback(const nav_msgs::Odometry& odom);
    void open_gripper(trajectory_msgs::JointTrajectory& posture);
    void closed_gripper(trajectory_msgs::JointTrajectory& posture);
    void pick(moveit::planning_interface::MoveGroupInterface& arm_group);
    void place(moveit::planning_interface::MoveGroupInterface& arm_group);
    void addCollisionObjects(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface);


public:
    ros::NodeHandle n_; 
    ros::Subscriber sub_move_;
    ros::Subscriber sub_pose_;
    ros::Publisher pub_grip_;

    bool simulation;

    int move_flag;
    int view_flag;

    float grip_force;
    float grip_position;

    geometry_msgs::Twist grip_msg;


    float tran_pose_x;
    float tran_pose_y;
    float tran_pose_z;
    float tran_quat_x;
    float tran_quat_y; 
    float tran_quat_z;
    float tran_quat_w;
    float tran_yaw;
};


void Manipulator::open_gripper(trajectory_msgs::JointTrajectory& posture)
{

    grip_position = 1.0;
    grip_msg.linear.x = grip_force;
    grip_msg.linear.y = grip_position;
    pub_grip_.publish(grip_msg);

}

void Manipulator::closed_gripper(trajectory_msgs::JointTrajectory& posture)
{

    grip_position = 0.0;
    grip_msg.linear.x = grip_force;
    grip_msg.linear.y = grip_position;
    pub_grip_.publish(grip_msg);

}

void Manipulator::pick(moveit::planning_interface::MoveGroupInterface& arm_group)//,
                       //geometry_msgs::Pose& obj_pose)
{
    std::vector<moveit_msgs::Grasp> grasps;
    grasps.resize(1);

    // set the place to grasp the object
    grasps[0].grasp_pose.header.frame_id = "base_link";  // is this fucking necessary???
    // TODO
    // Be advised here, the coordinate of the objects is under the manipulator ref-frame
    tf2::Quaternion grasp_orientation;
    grasp_orientation.setRPY(-M_PI / 2, 0, 0);
    grasps[0].grasp_pose.pose.orientation = tf2::toMsg(grasp_orientation);
    grasps[0].grasp_pose.pose.position.x = 0.0;
    grasps[0].grasp_pose.pose.position.y = 0.70;
    grasps[0].grasp_pose.pose.position.z = 0.30;

    // PRE
    grasps[0].pre_grasp_approach.direction.header.frame_id = "base_link";
    // Direction is set as positive x axis 
    // change this by considering maximum arm manipulability
    grasps[0].pre_grasp_approach.direction.vector.z = -1.0;
    grasps[0].pre_grasp_approach.min_distance = 0.095;
    grasps[0].pre_grasp_approach.desired_distance = 0.115;

    // POST
    grasps[0].post_grasp_retreat.direction.header.frame_id = "base_link";
    grasps[0].post_grasp_retreat.direction.vector.z = 1.0;
    grasps[0].post_grasp_retreat.min_distance = 0.1;
    grasps[0].post_grasp_retreat.desired_distance = 0.25;

    open_gripper(grasps[0].pre_grasp_posture);
    ROS_INFO("gripper shall be opened ");

    closed_gripper(grasps[0].grasp_posture);
    ROS_INFO("gripper shall be closed ");

    arm_group.setSupportSurfaceName("table2");
    arm_group.pick("object", grasps);
}



void Manipulator::place(moveit::planning_interface::MoveGroupInterface& arm_group)//,
                        //geometry_msgs::Pose& set_pose)
{
    std::vector<moveit_msgs::PlaceLocation> place_location;
    place_location.resize(1);

    // Setting place location pose

    place_location[0].place_pose.header.frame_id = "base_link";
    tf2::Quaternion orientation;
    orientation.setRPY(0, 0, M_PI / 2);
    place_location[0].place_pose.pose.orientation = tf2::toMsg(orientation);

    /* While placing it is the exact location of the center of the object. */
    place_location[0].place_pose.pose.position.x = -0.30;
    place_location[0].place_pose.pose.position.y = 0.30;
    place_location[0].place_pose.pose.position.z = 0.30;

    // Setting pre-place approach
    place_location[0].pre_place_approach.direction.header.frame_id = "base_link";
    /* Direction is set as negative z axis */
    place_location[0].pre_place_approach.direction.vector.z = -1.0;
    place_location[0].pre_place_approach.min_distance = 0.095;
    place_location[0].pre_place_approach.desired_distance = 0.115;

    // Setting post-grasp retreat
    place_location[0].post_place_retreat.direction.header.frame_id = "base_link";
    /* Direction is set as negative y axis */
    place_location[0].post_place_retreat.direction.vector.y = -1.0;
    place_location[0].post_place_retreat.min_distance = 0.1;
    place_location[0].post_place_retreat.desired_distance = 0.25;


    open_gripper(place_location[0].post_place_posture);

    arm_group.setSupportSurfaceName("table1");
    arm_group.place("object", place_location);
}



void Manipulator::flag_callback(const std_msgs::Int8& flag)
{
    if(flag.data == 1)
    {
        ROS_INFO("Received a flag message!");
        move_flag = 1;
    }
}

void Manipulator::obj_callback(const nav_msgs::Odometry& odom)
{
    if(odom.header.frame_id == "ok")
    {
        tran_pose_x = odom.pose.pose.position.x;
        tran_pose_y = odom.pose.pose.position.y;
        tran_pose_z = odom.pose.pose.position.z;
        tran_quat_x = odom.pose.pose.orientation.x;
        tran_quat_y = odom.pose.pose.orientation.y;
        tran_quat_z = odom.pose.pose.orientation.z;
        tran_quat_w = odom.pose.pose.orientation.w;
        tran_yaw = odom.twist.twist.angular.z;
        view_flag = 1;
    }
}

void Manipulator::addCollisionObjects(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface)
{
  // BEGIN_SUB_TUTORIAL table1
  //
  // Creating Environment
  // ^^^^^^^^^^^^^^^^^^^^
  // Create vector to hold 3 collision objects.
  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.resize(4);

  // Add the first table where the cube will originally be kept.
  collision_objects[0].id = "table1";
  collision_objects[0].header.frame_id = "base_link";

  /* Define the primitive and its dimensions. */
  collision_objects[0].primitives.resize(1);
  collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
  collision_objects[0].primitives[0].dimensions.resize(3);
  collision_objects[0].primitives[0].dimensions[0] = 0.20;
  collision_objects[0].primitives[0].dimensions[1] = 0.20;
  collision_objects[0].primitives[0].dimensions[2] = 0.20;

  /* Define the pose of the table. */
  collision_objects[0].primitive_poses.resize(1);
  collision_objects[0].primitive_poses[0].position.x = -0.40;
  collision_objects[0].primitive_poses[0].position.y = 0.40;
  collision_objects[0].primitive_poses[0].position.z = 0.10;
  // END_SUB_TUTORIAL

  collision_objects[0].operation = collision_objects[0].ADD;

  // BEGIN_SUB_TUTORIAL table2
  // Add the second table where we will be placing the cube.
  collision_objects[1].id = "table2";
  collision_objects[1].header.frame_id = "base_link";

  /* Define the primitive and its dimensions. */
  collision_objects[1].primitives.resize(1);
  collision_objects[1].primitives[0].type = collision_objects[1].primitives[0].BOX;
  collision_objects[1].primitives[0].dimensions.resize(3);
  collision_objects[1].primitives[0].dimensions[0] = 0.20;
  collision_objects[1].primitives[0].dimensions[1] = 0.20;
  collision_objects[1].primitives[0].dimensions[2] = 0.20;

  /* Define the pose of the table. */
  collision_objects[1].primitive_poses.resize(1);
  collision_objects[1].primitive_poses[0].position.x = 0;
  collision_objects[1].primitive_poses[0].position.y = 0.60;
  collision_objects[1].primitive_poses[0].position.z = 0.10;
  // END_SUB_TUTORIAL

  collision_objects[1].operation = collision_objects[1].ADD;

  // BEGIN_SUB_TUTORIAL object
  // Define the object that we will be manipulating
  collision_objects[2].header.frame_id = "base_link";
  collision_objects[2].id = "object";

  /* Define the primitive and its dimensions. */
  collision_objects[2].primitives.resize(1);
  collision_objects[2].primitives[0].type = collision_objects[1].primitives[0].BOX;
  collision_objects[2].primitives[0].dimensions.resize(3);
  collision_objects[2].primitives[0].dimensions[0] = 0.02;
  collision_objects[2].primitives[0].dimensions[1] = 0.02;
  collision_objects[2].primitives[0].dimensions[2] = 0.2;

  /* Define the pose of the object. */
  collision_objects[2].primitive_poses.resize(1);
  collision_objects[2].primitive_poses[0].position.x = 0.0;
  collision_objects[2].primitive_poses[0].position.y = 0.60;
  collision_objects[2].primitive_poses[0].position.z = 0.30;
  // END_SUB_TUTORIAL

  collision_objects[2].operation = collision_objects[2].ADD;

  collision_objects[3].header.frame_id = "base_link";
  collision_objects[3].id = "frame";

  /* Define the primitive and its dimensions. */
  collision_objects[3].primitives.resize(1);
  collision_objects[3].primitives[0].type = collision_objects[1].primitives[0].BOX;
  collision_objects[3].primitives[0].dimensions.resize(3);
  collision_objects[3].primitives[0].dimensions[0] = 0.8;
  collision_objects[3].primitives[0].dimensions[1] = 0.1;
  collision_objects[3].primitives[0].dimensions[2] = 0.8;

  /* Define the pose of the object. */
  collision_objects[3].primitive_poses.resize(1);
  collision_objects[3].primitive_poses[0].position.x = 0.0;
  collision_objects[3].primitive_poses[0].position.y = -0.40;
  collision_objects[3].primitive_poses[0].position.z = 0.40;
  // END_SUB_TUTORIAL

  collision_objects[3].operation = collision_objects[3].ADD;


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
    std::string end_effector_link = arm.getEndEffectorLink(); //get the end_effector link
    std::cout<<"end_effector_link: "<<end_effector_link<<std::endl;
    std::string reference_frame = "base_link"; // set the reference frame
    geometry_msgs::Pose now_pose = arm.getCurrentPose(end_effector_link).pose;
    std::cout<<"now Robot position: [x,y,z]: ["
             <<now_pose.position.x<<","<<now_pose.position.y<<","<<now_pose.position.z<<"]"
             <<std::endl;
    std::cout<<"now Robot orientation: [x,y,z,w]: ["
             <<now_pose.orientation.x<<","<<now_pose.orientation.y<<","<<now_pose.orientation.z<<","<<now_pose.orientation.w<<"]"
             <<std::endl;
    arm.setPoseReferenceFrame(reference_frame);
    arm.allowReplanning(true); // Evangelion 3.33: You Can(Not) Redo.
    // arm.setGoalJointTolerance(0.01);
    // arm.setGoalPositionTolerance(0.01); // set position torlerance
    // arm.setGoalOrientationTolerance(0.01);  // set orientation torlerance
    arm.setMaxAccelerationScalingFactor(0.2); // set max accelerations
    arm.setMaxVelocityScalingFactor(0.2);
    arm.setPlanningTime(45.0);

        
    ros::WallDuration(1.0).sleep();
    manipulator.pick(arm);

    manipulator.place(arm);



 
    ros::waitForShutdown();

    return 0;
}