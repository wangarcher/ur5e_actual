// program initialized in 210813
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
	ros::init(argc, argv, "plan_demo3");
	ros::AsyncSpinner spinner(1);
	spinner.start();

    moveit::planning_interface::MoveGroupInterface arm("manipulator");
  
    std::string end_effector_link = arm.getEndEffectorLink(); //get the end_effector link
    std::cout<<"end_effector_link: "<<end_effector_link<<std::endl;
   
    std::string reference_frame = "base_link"; // set the reference frame
    arm.setPoseReferenceFrame(reference_frame);

    arm.allowReplanning(true); // Evangelion 3.33: You Can(Not) Redo.
    arm.setGoalJointTolerance(0.001);
    arm.setGoalPositionTolerance(0.001); // set position torlerance
    arm.setGoalOrientationTolerance(0.01);  // set orientation torlerance
    arm.setMaxAccelerationScalingFactor(0.2); // set max accelerations
    arm.setMaxVelocityScalingFactor(0.2);

    geometry_msgs::Pose now_pose = arm.getCurrentPose(end_effector_link).pose;
    std::cout<<"now Robot position: [x,y,z]: ["<<now_pose.position.x<<","<<now_pose.position.y<<","<<now_pose.position.z<<"]"<<std::endl;
    std::cout<<"now Robot orientation: [x,y,z,w]: ["<<now_pose.orientation.x<<","<<now_pose.orientation.y<<","<<now_pose.orientation.z
       <<","<<now_pose.orientation.w<<"]"<<std::endl;
    const double delta = 0.02;

    std::vector<geometry_msgs::Pose> waypoints;
    geometry_msgs::Pose pose1;
    pose1.position.x = now_pose.position.x + delta;
	pose1.position.y = now_pose.position.y - delta;	
	pose1.position.z = now_pose.position.z - delta;
	pose1.orientation.x = now_pose.orientation.x;
	pose1.orientation.y = now_pose.orientation.y;
	pose1.orientation.z = now_pose.orientation.z;
	pose1.orientation.w = now_pose.orientation.w;
	waypoints.push_back(pose1);
    ROS_INFO("trajectory point insert");
    geometry_msgs::Pose pose2;
    pose2.position.x = now_pose.position.x + 2 * delta;
	pose2.position.y = now_pose.position.y - 2 * delta;	
	pose2.position.z = now_pose.position.z - 2 * delta;
	pose2.orientation.x = now_pose.orientation.x;
	pose2.orientation.y = now_pose.orientation.y;
	pose2.orientation.z = now_pose.orientation.z;
	pose2.orientation.w = now_pose.orientation.w;
	waypoints.push_back(pose2);

	// planning
	moveit_msgs::RobotTrajectory trajectory;
	const double jump_threshold = 0.0;
	const double eef_step = 0.002;
	double fraction = 0.0;
    int maxtries = 100;   // max attempts
    int attempts = 0;     // attempts done

    while(fraction < 1.0 && attempts < maxtries)
    {
        fraction = arm.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
        attempts++;
        
        if(attempts % 10 == 0)
            ROS_INFO("Still trying after %d attempts...", attempts);
    }
    
    if(fraction == 1)
    {   
        ROS_INFO("Path computed successfully. Moving the arm.");

	    // plan data
	    moveit::planning_interface::MoveGroupInterface::Plan plan;
	    plan.trajectory_ = trajectory;
	    // execute
	    arm.execute(plan);
        
        sleep(1);
    }
    else
    {
        ROS_INFO("Path planning failed with only %0.6f success after %d attempts.", fraction, maxtries);
    }
    
	ros::shutdown(); 
	return 0;
}
