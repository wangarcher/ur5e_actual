#include "ur5e_move_group_interface/ur5e_movement.h"
#include "ur5e_move_group_interface/mode_switcher.h"

bool servo_mode(ros::ServiceClient& controller_switch)
{
    controller_manager_msgs::SwitchController switcher;

    switcher.request.stop_controllers.push_back("scaled_pos_joint_traj_controller");
    switcher.request.stop_controllers.push_back("pos_joint_traj_controller");
    switcher.request.start_controllers.push_back("joint_group_vel_controller");

    switcher.request.strictness = 0;
    switcher.request.start_asap = false;
    switcher.request.timeout = 0.;

    controller_switch.call(switcher);

    return switcher.response.ok;
}

bool servo_to_admittance(ros::ServiceClient& controller_switch)
{
    controller_manager_msgs::SwitchController switcher;

    switcher.request.stop_controllers.push_back("joint_group_vel_controller");
    switcher.request.start_controllers.push_back("ur5_cartesian_velocity_controller_sim");

    switcher.request.strictness = 0;
    switcher.request.start_asap = false;
    switcher.request.timeout = 0.;

    controller_switch.call(switcher);

    return switcher.response.ok;
}

bool admittance_to_servo(ros::ServiceClient& controller_switch)
{
    controller_manager_msgs::SwitchController switcher;

    switcher.request.stop_controllers.push_back("ur5_cartesian_velocity_controller_sim");
    switcher.request.start_controllers.push_back("joint_group_vel_controller");

    switcher.request.strictness = 0;
    switcher.request.start_asap = false;
    switcher.request.timeout = 0.;

    controller_switch.call(switcher);

    return switcher.response.ok;
}