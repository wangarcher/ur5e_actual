#include "ros/ros.h"
#include "PIDController.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pid_controller_node");

  ros::NodeHandle nh;

  // Parameters
  std::string topic_arm_state;
  std::string topic_arm_command;

  std::vector<double> K_p;
  std::vector<double> K_i;
  std::vector<double> K_d;
  std::vector<double> workspace_limits;

  double arm_max_vel;
  double arm_acc_upper_limit;
  double arm_acc_lower_limit;
  double frequency;


  /// LOADING PARAMETERS FROM THE ROS SERVER
  if (!nh.getParam("topic_arm_state", topic_arm_state)) {
    ROS_ERROR("Couldn't retrieve the topic name for the state of the arm.");
    return -1;
  }
  if (!nh.getParam("topic_arm_command", topic_arm_command)) {
    ROS_ERROR("Couldn't retrieve the topic name for commanding the arm.");
    return -1;
  }

  /// ADMITTANCE PARAMETERS
  if (!nh.getParam("K_p", K_p)) {
    ROS_ERROR("Couldn't retrieve the desired mass of the arm.");
    return -1;
  }
  if (!nh.getParam("K_i", K_i)) {
    ROS_ERROR("Couldn't retrieve the desired damping of the arm.");
    return -1;
  }
  if (!nh.getParam("K_d", K_d)) {
    ROS_ERROR("Couldn't retrieve the desired stiffness of the coupling.");
    return -1;
  }

  /// SAFETY PARAMETERS
  if (!nh.getParam("workspace_limits", workspace_limits)) {
    ROS_ERROR("Couldn't retrieve the limits of the workspace.");
    return -1;
  }
  if (!nh.getParam("arm_max_vel", arm_max_vel)) {
    ROS_ERROR("Couldn't retrieve the max velocity for the arm.");
    return -1;
  }
  if (!nh.getParam("arm_acc_upper_limit", arm_acc_upper_limit)) {
    ROS_ERROR("Couldn't retrieve the max uppper acceleration for the arm.");
    return -1;
  }  
  if (!nh.getParam("arm_acc_lower_limit", arm_acc_lower_limit)) {
    ROS_ERROR("Couldn't retrieve the max lower acceleration for the arm.");
    return -1;
  }  
  if (!nh.getParam("frequency", frequency)) {
    ROS_ERROR("Couldn't retrieve the max acceleration for the arm.");
    return -1;
  }

  // Constructing the controller
  PIDController pid_controller(
    nh,
    frequency,
    topic_arm_command,
    topic_arm_state,
    K_p, K_i, K_d,
    workspace_limits,
    arm_max_vel, 
    arm_acc_upper_limit,
    arm_acc_lower_limit);

  // Running the controller
  pid_controller.run();

  return 0;
}