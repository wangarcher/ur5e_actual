#include "ur5e_move_group_interface/ur5e_movement.h"

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include <geometry_msgs/TwistStamped.h>
#include "ur5e_move_group_interface/mode_switcher.h"

#include <ur_dashboard_msgs/Load.h>
#include <ur_dashboard_msgs/RobotMode.h> 
#include <ur_dashboard_msgs/GetSafetyMode.h>
#include <std_srvs/Trigger.h>
int main(int argc, char **argv)
{
    ros::init(argc, argv, "service_test");
    ros::AsyncSpinner spinner(7);

    ros::NodeHandle nh;
    ros::ServiceClient zero_ftsensor_client_ = nh.serviceClient<std_srvs::Trigger>("/ur_hardware_interface/zero_ftsensor") ;
    std_srvs::Trigger ft_trigger;

    zero_ftsensor_client_.call(ft_trigger);
    std::cout << "over" << std::endl;
    ros::waitForShutdown();
    return 0;
}