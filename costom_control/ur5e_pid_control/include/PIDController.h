#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H

#include "ros/ros.h"
#include "geometry_msgs/WrenchStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "nav_msgs/Odometry.h"
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_listener.h>

#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Geometry"
#include "eigen3/Eigen/Dense"

#include "std_msgs/Float32.h"

#include "std_srvs/Trigger.h"

#include <ur_dashboard_msgs/RobotMode.h> 


using namespace Eigen;

typedef Matrix<double, 7, 1> Vector7d;
typedef Matrix<double, 6, 1> Vector6d;
typedef Matrix<double, 6, 6> Matrix6d;
typedef Matrix<double, 3, 1> Vector3d;
typedef Matrix<double, 3, 3> Matrix3d;
typedef Matrix<double, 6, 6> Matrix6d;
typedef Matrix<double, 3, 6> Matrix3_6d;
typedef Matrix<double, 6, 3> Matrix6_3d;


class PIDController
{
protected:
  ////// ROS VARIABLES:
  // A handle to the node in ros
  ros::NodeHandle nh_;
  // Rate of the run loop
  ros::Rate loop_rate_;

  ///// Subscribers:
  // Subscriber for the arm state
  ros::Subscriber sub_arm_state_;
 
  ros::Subscriber sub_now_equilibrium_;

  ////// Publishers:
  // Publisher for the twist of arm endeffector
  ros::Publisher pub_arm_cmd_;

  ros::Subscriber robot_mode_sub_;


  int count;
  int ur5e_mode_;

  /////// ADMITTANCE PARAMETERS:
  // M_a_ -> Desired mass of arm
  // D_a_ -> Desired damping of arm
  // K_ -> Desired Stiffness of the coupling
  Matrix6d K_p_, K_i_, K_d_;
  // equilibrium position of the coupling spring
  Vector3d equilibrium_position_;
  // equilibrium orientation of the coupling spring
  Quaterniond equilibrium_orientation_;

  // OUTPUT COMMANDS
  // final arm desired velocity 
  Vector6d arm_desired_twist_final_;
  Vector6d last_arm_desired_twist_final_;

  Vector6d last_error_;

  // Vector6d total_error_;
  // Vector6d last_arm_real_twist_;

  // limiting the workspace of the arm
  Vector6d workspace_limits_;
  double arm_max_vel_;
  double arm_acc_upper_limit_;
  double arm_acc_lower_limit_;
  double frequency;
  double eps_;

  ////// STATE VARIABLES:
  // Arm state: position, orientation, and twist (in "ur5_arm_base_link")
  Vector3d arm_real_position_;
  Quaterniond arm_real_orientation_;
  Vector6d arm_real_twist_;
  
  ////// LOWPASS_FILTER
  Vector7d x_0, x_1, x_2, x_3;
  Vector7d y_0, y_1, y_2, y_3;
  Vector7d mid_x, mid_y;

  Vector3d cx_0, cx_1, cx_2, cx_3;
  Vector3d cy_0, cy_1, cy_2, cy_3;
  Vector3d cmid_y;

  // Transform from base_link to world
  Matrix6d rotation_base_;

  // TF:
  // Listeners
  tf::TransformListener listener_ft_;
  tf::TransformListener listener_control_;
  tf::TransformListener listener_arm_;

  // 判断是否所有坐标系矩阵建立
  bool transformation;

  // Initialization
  void wait_for_transformations();

  // Control
  void compute_pid();

  // Callbacks
  void state_arm_callback(const nav_msgs::OdometryConstPtr msg);
  void now_equilibrium_callback(const geometry_msgs::PoseStampedConstPtr msg);
  void robot_mode_callback(const ur_dashboard_msgs::RobotMode::ConstPtr& msg);

  // Util
  bool get_rotation_matrix(Matrix6d & rotation_matrix,tf::TransformListener & listener,std::string from_frame,  std::string to_frame);

  void limit_to_workspace();
  void kalman_filter();
  void lowpass_filter();
  void send_commands_to_robot();


public:
  PIDController(ros::NodeHandle &n, double frequency,
                       std::string cmd_topic_arm,
                       std::string state_topic_arm,
                       std::vector<double> K_p,
                       std::vector<double> K_i,
                       std::vector<double> K_d,
                       std::vector<double> workspace_limits,
                       double arm_max_vel,
                       double arm_acc_upper_limit,
                       double arm_acc_lower_limit);
  void run();
};

#endif // PIDCONTROLLER_H
