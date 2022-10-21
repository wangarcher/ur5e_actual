// a simple planning demo, initialized in 210813
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <vector>
#include <iostream>
#include <tf/transform_listener.h>
 
int main(int argc, char **argv)
{
    
    ros::init(argc, argv, "plan_demo2"); 
    ros::AsyncSpinner spinner(1); 
    spinner.start(); 
 
    moveit::planning_interface::MoveGroupInterface arm("manipulator");
    tf::TransformListener listener;
    tf::StampedTransform transform;

    while(1)
    {
        try
        {
            listener.lookupTransform("/base_link","/tool0", ros::Time(0), transform);
            std::cout << transform.getOrigin().z() << std::endl;
        }
        catch(tf::TransformException &ex)
        {
            ROS_ERROR("%s", ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }

    }
    

 
    ros::shutdown();
 
    return 0;
}