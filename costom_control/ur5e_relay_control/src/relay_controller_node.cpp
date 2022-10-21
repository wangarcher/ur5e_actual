#include "ros/ros.h"
#include "RelayController.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "relay_controller_node");

  ros::NodeHandle nh;
  double frequency = 1000.0;
  // Parameters
  std::string topic_arm_command;


  if (!nh.getParam("topic_arm_command", topic_arm_command)) {
    ROS_ERROR("Couldn't retrieve the topic name for commanding the arm.");
    return -1;
  }

  // Constructing the controller
  RelayController relay_controller(
    nh,
    frequency,
    topic_arm_command);

  // Running the controller
  relay_controller.run();

  return 0;
}
