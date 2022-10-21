// a pick ang place demo, initialized in 211124
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <iostream>
#include <math.h>
#include <ros/ros.h>

#include "general_pp.h"


int main(int argc, char **argv)
{
    
    ros::init(argc, argv, "project_plan");
    Manipulator manipulator;
    ros::AsyncSpinner spinner(7);
	spinner.start();


    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

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

    // while(manipulator.n_.ok())
    // {
    //     if(manipulator.controller_flag_ == 1)
    //     {
    //         manipulator.tracking();
    //     }
    //     if(manipulator.controller_flag_ == 2)
    //     {
    //         manipulator.stablize();
    //     }
    // }

    manipulator.stablize();


    ros::waitForShutdown();

    return 0;
}