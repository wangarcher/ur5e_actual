#include "RelayController.h"

RelayController::RelayController(ros::NodeHandle &n,
    double frequency,
    std::string topic_arm_command) :
  nh_(n), loop_rate_(frequency)
{
  ///// Subscribers

  sub_now_equilibrium_ = nh_.subscribe("/now_equilibrium", 1,
                                      &RelayController::now_equilibrium_callback, this);
  ////// Publishers
  pub_arm_cmd_ = nh_.advertise<geometry_msgs::Pose>(topic_arm_command, 1);


  x_0.setZero();
  x_1.setZero();
  x_2.setZero();
  x_3.setZero();
  x_4.setZero();
  x_5.setZero();
  x_6.setZero();

  y_0.setZero();
  y_1.setZero();
  y_2.setZero();
  y_3.setZero();
  y_4.setZero();
  y_5.setZero();
  y_6.setZero();

  mid_x.setZero();
  mid_y.setZero();

}




void RelayController::now_equilibrium_callback(const geometry_msgs::PoseConstPtr msg)
{
  equilibrium_position_(0) = msg->position.x;
  equilibrium_position_(1) = msg->position.y;
  equilibrium_position_(2) = msg->position.z;

  equilibrium_orientation_.coeffs() << msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w;

  x_0 = x_1; x_1 = x_2; x_2 = x_3;
  y_0 = y_1; y_1 = y_2; y_2 = y_3;
  mid_x.topRows(3) = equilibrium_position_;
  
  // mid_x.bottomRows(4) << equilibrium_orientation_.w(),
  //                        equilibrium_orientation_.x(),
  //                        equilibrium_orientation_.y(),
  //                        equilibrium_orientation_.z();
  mid_x.bottomRows(4) << msg->orientation.w,
                         msg->orientation.x,
                         msg->orientation.y,
                         msg->orientation.z;                      
  x_3 = mid_x;
  mid_y = (x_0 + x_3) + 3.0 * (x_1 + x_2) + 596.182 * y_0 - 2050.265 * y_1 + 2372.446 * y_2;
  y_3 = mid_y / 926.364;

  x_4 = x_5; x_5 = x_6;
  y_4 = y_5; y_5 = y_6;

  x_6 = y_3;
  y_6 = 0.98968746 * x_6 - 1.96392245 * x_5 + 0.98968746 * x_4 + 1.96392245 * y_5 - 0.97937493 * y_4;
}

///////////////////////////////////////////////////////////////
///////////////////// Control Loop ////////////////////////////
///////////////////////////////////////////////////////////////
void RelayController::run() {

  ROS_INFO("Running the relay control loop .................");

  while (nh_.ok()) {


    // Copy commands to messages
    send_commands_to_robot();


    ros::spinOnce();
    loop_rate_.sleep();
  }


}


///////////////////////////////////////////////////////////////
//////////////////// COMMANDING THE ROBOT /////////////////////
///////////////////////////////////////////////////////////////
void RelayController::send_commands_to_robot() {
  geometry_msgs::Pose arm_pose_cmd;

  arm_pose_cmd.position.x  = y_6(0);
  arm_pose_cmd.position.y  = y_6(1);
  arm_pose_cmd.position.z  = y_6(2);
  arm_pose_cmd.orientation.x = y_6(4);
  arm_pose_cmd.orientation.y = y_6(5);
  arm_pose_cmd.orientation.z = y_6(6);
  arm_pose_cmd.orientation.w = y_6(3);
  pub_arm_cmd_.publish(arm_pose_cmd);
}
