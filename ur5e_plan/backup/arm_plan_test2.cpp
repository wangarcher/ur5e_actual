// a simple planning demo, initialized in 210813
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <vector>
#include <iostream>
 
int main(int argc, char **argv)
{
    
    ros::init(argc, argv, "plan_demo2"); 
    ros::AsyncSpinner spinner(1); 
    spinner.start(); 
 
    moveit::planning_interface::MoveGroupInterface arm("manipulator");
    
    arm.setGoalJointTolerance(0.001); // differ tolerance
    arm.setMaxAccelerationScalingFactor(0.2); // max velocity
    arm.setMaxVelocityScalingFactor(0.2);

    arm.setNamedTarget("home1"); // home, all set to zero
    arm.move();
    sleep(5);
 
    arm.setNamedTarget("qr_start"); //qr_state, for the best overlook view
    arm.move();
    sleep(2);
 
    ros::shutdown();
 
    return 0;
}