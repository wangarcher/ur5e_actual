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

      if(flag_mode == 0)
      {
          excuting = true;
          std::cout << "Manipulator go back "  << std::endl;
          pause_program.call(ur5e_trigger);
          operating_change = false;
          ur_dashboard_msgs::Load goback_load;
          goback_load.request.filename = "go_back.urp";
          load_program.call(goback_load);
          sleep(1);
          start_program.call(ur5e_trigger);
          sleep(3);
          // power_off.call(ur5e_trigger);
            flag_mode = -1;
          excuting = false;
      }
      else if(flag_mode == 1)
      {
          excuting = true;
          // package_state = 0;
          // radio_state = 0;
          std::cout << "Manipulator Preparing ... "  << std::endl;
          operating_change = false;
          ur_dashboard_msgs::Load init_load;
          init_load.request.filename = "init.urp";
          default_start(&power_on,&break_release);
          sleep(1);
          load_program.call(init_load);
          sleep(1);
          start_program.call(ur5e_trigger);
          sleep(3);
          default_program_load(&load_program,&start_program);
          sleep(1);
          // servo_mode(controller_switch);
          flag_mode = -1;
          excuting = false;
      }
      else if(flag_mode == 2)
      {
        servo_mode(controller_switch);
        flag_mode = -1;
        excuting = false;
      }

      // if(operating_mode == true)
      // {
      //     flag_mode = 0;
      //     continue;
      // }

      ur_dashboard_msgs::GetSafetyMode mode_get_tool;
      ur_dashboard_msgs::Load pose_reset;

      // std::cout << "Current flag mode :" << flag_mode << std::endl;
      
      switch(flag_mode)
      {
//         case 0:
//           servo_info_pub.publish(servo_info_ptr);
//           break;
//         case 1:
//           if(controller_switch_flag)
//           {
//             std::cout << "admittance control start ..." << std::endl;
//             servo_to_admittance(controller_switch);
//             zero_ftsensor_client.call(ur5e_trigger);    
//             controller_switch_flag = false;        
//           }
//           admittance_info_pub.publish(servo_info_ptr);
//           break;
//         case 2:  // force the arm go back to initial position
//           pose_reset.request.filename = "pose_reset.urp";  // name should be changed !!!
//           load_program.call(pose_reset);
//           start_program.call(ur5e_trigger);
//           sleep(3);
//           default_program_load(&load_program,&start_program);
//           radio_state = 0;
//           package_state = 0; 
//           flag_mode = 0;
//           break;
//         case 3:  // load PACKAGE_SEQ_0
// #ifdef PACKAGE_SEQ_0
//           if(package_state == 0)
//           {
//             package_state = 1;
//             excuting = true;
//             pause_program.call(ur5e_trigger);
//             action_m.request.filename = "package_action_0.urp";
//             load_program.call(action_m);
//             sleep(1);
//             start_program.call(ur5e_trigger);
//             sleep(16);
//             default_program_load(&load_program,&start_program);
//             sleep(1);
//             std::cout << "please relase the gripper " << std::endl;
//             excuting = false;
//           }
//           else
//           {
//             std::cout << "action denied,please reset to initial pose or continue the following sequence" << std::endl;
//             std::cout << "Be careful, operator" << std::endl;
//           }
// #endif
//           flag_mode = 0;
//           break;
//         case 4: // load PACKAGE_SEQ_1
// #ifdef PACKAGE_SEQ_1
//           if(package_state == 1)
//           {
//             package_state = 0;
//             excuting = true;
//             pause_program.call(ur5e_trigger);
//             action_m.request.filename = "package_action_1.urp";
//             load_program.call(action_m);
//             sleep(1);
//             start_program.call(ur5e_trigger);
//             sleep(9);
//             default_program_load(&load_program,&start_program);
//             sleep(1);
//             std::cout << "Package sequence done!" << std::endl;
//             excuting = false;
//           }
//           else
//           {
//             std::cout << "action denied,please reset to initial pose or continue the following sequence" << std::endl;
//             std::cout << "Be careful, operator" << std::endl;
//           }
// #endif
//           flag_mode = 0;
//           break;
        case 5: // recovery mode 
          get_safety_mode.call(mode_get_tool);
          if(mode_get_tool.response.success == false)
          {
            std::cout << "unable to get safety mode !" << std::endl;
            break;
          }
          switch (mode_get_tool.response.safety_mode.mode)
          {
          case 1:  // NORMAL
            default_start(&power_on,&break_release);
            break;
          case 2:  // REDUCED
            break;
          case 3:  // PROTECTIVE_STOP
            unlock_protective_stop.call(ur5e_trigger);
            default_program_load(&load_program,&start_program);
            break;           
          default:
            solve_safety_problem.call(ur5e_trigger);
            while(ur5e_mode!=W_POWER_OFF);
            default_start(&power_on,&break_release);
            default_program_load(&load_program,&start_program);
            break;
          }
          flag_mode = 0;
          break;
//         case ADMITTANCE_DIS:
//           admittance_to_servo(controller_switch);
//           flag_mode = 0;
//           break;
//         case 8:  // load RADIO_SEQ_0
// #ifdef RADIO_SEQ_0
//           if(radio_state == 0)
//           {
//             radio_state = 1;
//             excuting = true;
//             pause_program.call(ur5e_trigger);
//             action_m.request.filename = "radio_action_0.urp";
//             load_program.call(action_m);
//             sleep(1);
//             start_program.call(ur5e_trigger);
//             sleep(9);
//             default_program_load(&load_program,&start_program);
//             sleep(1);
//             std::cout << "please grasp and hold tight " << std::endl;
//             excuting = false;
//           }
//           else
//           {
//             std::cout << "action denied,please reset to initial pose or continue the following sequence" << std::endl;
//             std::cout << "Be careful, operator" << std::endl;
//           }
// #endif
//           flag_mode = 0;
//           break;
//         case 9:  // load RADIO_SEQ_1
// #ifdef RADIO_SEQ_1
//           if(radio_state == 1)
//           {
//             radio_state = 2;
//             excuting = true;
//             pause_program.call(ur5e_trigger);
//             action_m.request.filename = "radio_action_1.urp";
//             load_program.call(action_m);
//             sleep(1);
//             start_program.call(ur5e_trigger);
//             sleep(16);
//             default_program_load(&load_program,&start_program);
//             sleep(1);
//             std::cout << "please relase the gripper " << std::endl;
//             excuting = false;
//           }
//           else
//           {
//             std::cout << "action denied,please reset to initial pose or continue the following sequence" << std::endl;
//             std::cout << "Be careful, operator" << std::endl;
//           }
// #endif
//           flag_mode = 0;
//           break;
//         case 10: // load RADIO_SEQ_2
// #ifdef RADIO_SEQ_2
//           if(radio_state == 2)
//           {
//             radio_state = 0;
//             excuting = true;
//             pause_program.call(ur5e_trigger);
//             action_m.request.filename = "radio_action_2.urp";
//             load_program.call(action_m);
//             sleep(1);
//             start_program.call(ur5e_trigger);
//             sleep(16);
//             default_program_load(&load_program,&start_program);
//             sleep(1);
//             std::cout << " Radio sequence done! " << std::endl;
//             excuting = false;
//           }
//           else
//           {
//             std::cout << "action denied,please reset to initial pose or continue the following sequence" << std::endl;
//             std::cout << "Be careful, operator" << std::endl;
//           }
// #endif
//           flag_mode = 0;
//           break;
//         default:
//           flag_mode = 0;
//           break;
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

  // if(flag_temp == OPREATING_SWITCH && excuting == false)
  // {
  //   // operating_mode = !operating_mode;
  //   operating_change = true;
  //   res.success = 1;
  //   return true;
  // }
  //  // only in servo mode, we can interrupt and change ur5e to excute other mission 
  //  // and only in admittance control mode, we need switch to servo manually
  // if((flag_mode == 0 && flag_temp != ADMITTANCE_DIS) || (flag_mode == 1 && flag_temp == ADMITTANCE_DIS) )
  // {
  //   flag_mode = flag_temp;
  //   res.success = 1;
  //   if(flag_temp == 1)
  //   {
  //     controller_switch_flag = true;
  //   }
  // }
  
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
    control_load.request.filename = "ros0810.urp";
    load_program->call(control_load);
    start_program->call(ur5e_trigger);

    if(control_load.response.success == false)
    {
      std::cout << control_load.response.answer <<"failed to load ros0810.urp" << std::endl;
      return 1;
    }

    return 0;
}
