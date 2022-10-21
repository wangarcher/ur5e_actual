#include <controller_manager_msgs/SwitchController.h>

bool servo_mode(ros::ServiceClient& controller_switch);
bool servo_to_admittance(ros::ServiceClient& controller_switch);
bool admittance_to_servo(ros::ServiceClient& controller_switch);