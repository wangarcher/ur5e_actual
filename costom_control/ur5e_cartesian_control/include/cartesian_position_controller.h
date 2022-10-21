#ifndef CARTESIAN_POSITION_CONTROLLER_H
#define CARTESIAN_POSITION_CONTROLLER_H

#include <ros/node_handle.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <hardware_interface/joint_command_interface.h>
#include <controller_interface/controller.h>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolvervel_pinv_givens.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include "kinematic_chain_controller_base.h"
#include <realtime_tools/realtime_publisher.h>

namespace controller_interface
{

/** \brief This class implements a ROS control cartesian velocity
 * controller. Its base class implements the core
 * of the controller.
 */

template<typename T>
class CartesianPositionControllerBase:
    public KinematicChainControllerBase<T>
{
public:
  CartesianPositionControllerBase() {}
  ~CartesianPositionControllerBase() {}

  /** \brief The init function is called to initialize the controller from a
   * non-realtime thread with a pointer to the hardware interface, itself,
   * instead of a pointer to a RobotHW.
   *s
   * \param robot The specific hardware interface used by this controller.
   *
   * \param n A NodeHandle in the namespace from which the controller
   * should read its configuration, and where it should set up its ROS
   * interface.
   *
   * \returns True if initialization was successful and the controller
   * is ready to be started.
   */
  bool init(T *robot, ros::NodeHandle &n);

  /** \brief This is called from within the realtime thread just before the
   * first call to \ref update
   *
   * \param time The current time
   */
  void starting(const ros::Time& time);

  /*!
   * \brief Issues commands to the joint. Called at regular intervals
   */
  void update(const ros::Time& time, const ros::Duration& period);

  /*!
   * \brief Subscriber's callback function
   */
  void command_cart_pos(const geometry_msgs::PoseConstPtr &msg);

  /** \brief Write current commands to the hardware interface
   */
  virtual void writePositionCommands(const ros::Duration& period) = 0;

protected:
  ros::Subscriber sub_command_; // Interface to external commands

  KDL::Twist x_dt_des_;      // Desired end-effector velocity
  KDL::JntArray q_dt_cmd_;      // Desired joint velocity
  KDL::JntArray q_cmd_;
  KDL::JntArray q_min_;
  KDL::JntArray q_max_;

  boost::shared_ptr<KDL::ChainIkSolverVel> ik_vel_solver_;
  boost::shared_ptr<KDL::ChainIkSolverPos> ik_pos_solver_;

  ros::Time last_publish_time_;
  double publish_rate_;

  KDL::FrameVel x_dot_;
  KDL::Frame x_;
  KDL::Frame x_des_;
   
  int count;

  boost::shared_ptr<KDL::ChainFkSolverVel> fk_vel_solver_;
  boost::shared_ptr<KDL::ChainFkSolverPos> fk_pos_solver_;
};

/** \brief Two different instantiations of the CartesianVelocityControllerBase
 * are provided here:
 * - CartesianVelocityController: with a VelocityJointInterface
 * - CartesianVelocityControllerSim: with a PositionJointInterface
 *   (for gazebo)
 */

/** The controller instance for the real hardware
 * */
class CartesianPositionController :
  public CartesianPositionControllerBase<
        hardware_interface::PositionJointInterface> {
    /**
     * Write velocity commands through a velocity interface
     */
    void writePositionCommands(const ros::Duration& period);
};

class CartesianPositionControllerSim :
  public CartesianPositionControllerBase<
        hardware_interface::PositionJointInterface> {

  /**
   * Write Position commands through a position interface
   */
  void writePositionCommands(const ros::Duration& period);
};


} // namespace controller_interface


#endif
