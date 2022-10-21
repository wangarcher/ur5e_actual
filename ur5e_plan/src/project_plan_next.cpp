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
#include <iostream>
#include <mission_mode/mode_switch.h>
#include <std_srvs/Trigger.h>


int main(int argc, char **argv)
{
    
    ros::init(argc, argv, "project_plan");
    ros::NodeHandle n;
    ros::AsyncSpinner spinner(2);
	spinner.start();

    bool place_flag = true;
    bool done_flag = true;
    float grip_force = 0;
    float grip_position = 0;
    moveit::planning_interface::MoveGroupInterface arm("manipulator");
    
    arm.allowReplanning(true); // Evangelion 3.33: You Can(Not) Redo.
    arm.setGoalJointTolerance(0.001);
    arm.setGoalPositionTolerance(0.001); // set position torlerance
    arm.setGoalOrientationTolerance(0.01);  // set orientation torlerance
    arm.setMaxAccelerationScalingFactor(0.2); // set max accelerations
    arm.setMaxVelocityScalingFactor(0.2);
    

    ros::Rate loop_rate(10);
    ros::Publisher pub = n.advertise<geometry_msgs::Twist>("/grip_control", 1);
    ros::ServiceClient vnc_client = n.serviceClient<mission_mode::mode_switch>("/arm_mode_switch");

    while(n.ok() && done_flag)
    {
        if(place_flag)
        {
            std::cout<<"in the last plan"<<std::endl;
            arm.setNamedTarget("ground_place"); //qr_state, for the best overlook view
            arm.move();
            done_flag = false;
            place_flag = false;
            sleep(1);
            grip_force = 0.7;
            grip_position = 0.9;
            geometry_msgs::Twist msgOut;
            msgOut.linear.x = grip_force;
            msgOut.linear.y = grip_position;
            pub.publish(msgOut);
            sleep(1);

            mission_mode::mode_switch goback_sig;
            goback_sig.request.mode_flag = 0;
            vnc_client.call(goback_sig);
            sleep(3);
            ROS_WARN("Finish call Goback");

        }
        loop_rate.sleep();

    }
    ros::waitForShutdown();

    return 0;
}