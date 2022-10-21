#ifndef RELAYCONTROLLER_H
#define RELAYCONTROLLER_H

#include "ros/ros.h"
#include "geometry_msgs/WrenchStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/Pose.h"

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

class RelayController
{
protected:
  ////// ROS VARIABLES:
  // A handle to the node in ros
  ros::NodeHandle nh_;
  // Rate of the run loop
  ros::Rate loop_rate_;

  ros::Subscriber sub_now_equilibrium_;

  ////// Publishers:
  // Publisher for the twist of arm endeffector
  ros::Publisher pub_arm_cmd_;


  Vector3d equilibrium_position_;
  // equilibrium orientation of the coupling spring
  Quaterniond equilibrium_orientation_;


  Vector7d x_0, x_1, x_2, x_3, x_4, x_5, x_6;
  Vector7d y_0, y_1, y_2, y_3, y_4, y_5, y_6;
  Vector7d mid_x, mid_y;


  // Callbacks
  void now_equilibrium_callback(const geometry_msgs::PoseConstPtr msg);

  void send_commands_to_robot();


public:
  RelayController(ros::NodeHandle &n, double frequency,
                       std::string cmd_topic_arm);
  void run();
};

#endif // RELAYCONTROLLER_H

