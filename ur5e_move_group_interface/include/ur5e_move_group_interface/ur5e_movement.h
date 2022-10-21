#include <ros/ros.h>
#include "mission_mode/mode_switch.h"

#define ADMITTANCE_DIS 7
#define OPREATING_SWITCH 11
  // int8 NO_CONTROLLER=-1
  // int8 DISCONNECTED=0
  // int8 CONFIRM_SAFETY=1
  // int8 BOOTING=2
  // int8 POWER_OFF=3
  // int8 POWER_ON=4
  // int8 IDLE=5
  // int8 BACKDRIVE=6
  // int8 RUNNING=7
  // int8 UPDATING_FIRMWARE=8

#define W_CONFIRM_SAFETY 1
#define W_POWER_OFF 3
#define W_POWER_ON 4
#define W_IDLE 5
#define W_RUNNING 7

#define PACKAGE_SEQ_0
#define PACKAGE_SEQ_1

#define RADIO_SEQ_0
#define RADIO_SEQ_1
#define RADIO_SEQ_2

bool arm_mode_switch_handle(mission_mode::mode_switch::Request &req, mission_mode::mode_switch::Response &res);
void default_start(ros::ServiceClient* power_on,ros::ServiceClient* break_release);
bool default_program_load(ros::ServiceClient* load_program,ros::ServiceClient* start_program);