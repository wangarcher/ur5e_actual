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

int flag_mode = -1;
bool controller_switch_flag = false;
int ur5e_mode = W_POWER_OFF;

bool operating_mode = false;
bool operating_change = false;

int radio_state = 0; // 0 stands for not doing any thing yet or mission done, 1 stands for ready to grasp, 2 stands for ready to putdown
int package_state = 0; // 0 stands for not doing any thing yet or mission done, 1 stands for ready to putdown

bool excuting = false;
void robot_mode_callback(const ur_dashboard_msgs::RobotMode::ConstPtr& msg);

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ur5e_move_group_interface");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();


    // read information from remote controller
    
    ros::Subscriber robot_mode_sub = nh.subscribe("/ur_hardware_interface/robot_mode",1,robot_mode_callback);
    // TODO: add more subscriber 
    
    ros::ServiceServer switch_service = nh.advertiseService("/arm_mode_switch",arm_mode_switch_handle);
    ros::ServiceClient controller_switch = nh.serviceClient<controller_manager_msgs::SwitchController>("/controller_manager/switch_controller");
    ros::ServiceClient load_program = nh.serviceClient<ur_dashboard_msgs::Load>("/ur_hardware_interface/dashboard/load_program");
    ros::ServiceClient start_program = nh.serviceClient<std_srvs::Trigger>("/ur_hardware_interface/dashboard/play");
    ros::ServiceClient pause_program = nh.serviceClient<std_srvs::Trigger>("/ur_hardware_interface/dashboard/pause");
    ros::ServiceClient zero_ftsensor_client = nh.serviceClient<std_srvs::Trigger>("/ur_hardware_interface/zero_ftsensor");
    // For recovery
    ros::ServiceClient get_safety_mode = nh.serviceClient<ur_dashboard_msgs::GetSafetyMode>("/ur_hardware_interface/dashboard/get_safety_mode");
    ros::ServiceClient break_release = nh.serviceClient<std_srvs::Trigger>("/ur_hardware_interface/dashboard/brake_release");
    ros::ServiceClient close_safety_popup = nh.serviceClient<std_srvs::Trigger>("/ur_hardware_interface/dashboard/close_safety_popup");
    ros::ServiceClient power_on = nh.serviceClient<std_srvs::Trigger>("/ur_hardware_interface/dashboard/power_on");
    ros::ServiceClient power_off = nh.serviceClient<std_srvs::Trigger>("/ur_hardware_interface/dashboard/power_off");
    ros::ServiceClient unlock_protective_stop = nh.serviceClient<std_srvs::Trigger>("/ur_hardware_interface/dashboard/unlock_protective_stop");
    // For Fatal Error
    ros::ServiceClient solve_safety_problem = nh.serviceClient<std_srvs::Trigger>("/ur_hardware_interface/dashboard/restart_safety");

    controller_manager_msgs::SwitchController switcher;
    
    static const std::string PLANNING_GROUP = "manipulator";
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);   
    //For adding collision parts of the land shaker
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    // add neccessary collision object(collision parts of the land shaker)

    std_srvs::Trigger ur5e_trigger;
    ur_dashboard_msgs::Load action_m;

    ros::Rate r(100);
    while(ros::ok())
    {
      // std::cout << flag_mode << std::endl;
      if(flag_mode == 1)
      {
        default_program_load(&load_program,&start_program);
        sleep(2);
        flag_mode = -1;
      }
      if(flag_mode == 7)
      {
        ur_dashboard_msgs::Load gripper_open;
        gripper_open.request.filename = "gripper_open.urp";
        load_program.call(gripper_open);
        sleep(1);
        start_program.call(ur5e_trigger);
        sleep(7);
        default_program_load(&load_program,&start_program);
        sleep(2);
        flag_mode = -1;
      }
      if(flag_mode == 8)
      {
        ur_dashboard_msgs::Load gripper_close;
        gripper_close.request.filename = "gripper_close.urp";
        load_program.call(gripper_close);
        sleep(1);
        start_program.call(ur5e_trigger);
        sleep(7);
        default_program_load(&load_program,&start_program);
        sleep(2);
        flag_mode = -1;
      }
    
      ros::spinOnce();
      r.sleep();
    }
    return 0;
}



bool arm_mode_switch_handle(mission_mode::mode_switch::Request &req, mission_mode::mode_switch::Response &res)
{
  int flag_temp;
  flag_temp = req.mode_flag;

  std::cout << flag_temp << std::endl;
  res.success = 1;
  flag_mode = flag_temp;

  return true;
}

void robot_mode_callback(const ur_dashboard_msgs::RobotMode::ConstPtr& msg)
{
  ur5e_mode = msg->mode;
}

void default_start(ros::ServiceClient* power_on,ros::ServiceClient* break_release)
{
  std_srvs::Trigger ur5e_trigger;
    if(ur5e_mode == W_POWER_OFF)
    {
      power_on->call(ur5e_trigger);
      while(ur5e_mode!= W_IDLE)
        ;
    }
    
    if(ur5e_mode == W_IDLE)
    {
      break_release->call(ur5e_trigger);
      while(ur5e_mode!= W_RUNNING)
        ;
    }
}

bool default_program_load(ros::ServiceClient* load_program,ros::ServiceClient* start_program)
{
    std_srvs::Trigger ur5e_trigger;
    ur_dashboard_msgs::Load control_load;
    control_load.request.filename = "ur_robot_driver.urp";
    load_program->call(control_load);
    start_program->call(ur5e_trigger);

    if(control_load.response.success == false)
    {
      std::cout << control_load.response.answer <<"failed to load ur_robot_driver.urp" << std::endl;
      return 1;
    }

    return 0;
}
