#include <pluginlib/class_list_macros.h>
#include "kinematic_chain_controller_base.h"
#include "cartesian_position_controller.h"
#include "kdl_conversions/kdl_msg.h"

namespace controller_interface
{
/** \brief Initialize the kinematic chain for kinematics-based computation.
 *
 */
template<typename T>
bool CartesianPositionControllerBase<T>::init(
    T *robot, ros::NodeHandle &n) {

  count = 1;
  // KDL
  KinematicChainControllerBase<T>::init(robot, n);
  Eigen::Matrix<double, 6, 1> L;
  L(0) = 1,0; 
  L(1) = 1.0; 
  L(2) = 1.0; 
  L(3) = 0.01; 
  L(4) = 0.01; 
  L(5) = 0.01; 

  ik_vel_solver_.reset(new KDL::ChainIkSolverVel_pinv_givens(this->kdl_chain_));
  fk_vel_solver_.reset(new KDL::ChainFkSolverVel_recursive(this->kdl_chain_));
  fk_pos_solver_.reset(new KDL::ChainFkSolverPos_recursive(this->kdl_chain_));
  ik_pos_solver_.reset(new KDL::ChainIkSolverPos_LMA(this->kdl_chain_, L, 1.0e-5, 5000, 1.0e-5));


  // get publishing period
  if (!n.getParam("publish_rate", publish_rate_)){
      ROS_ERROR("Parameter 'publish_rate' not set");
      return false;
  }


  // Topics
  sub_command_ = n.subscribe("command_cart_pos", 5,
                         &CartesianPositionControllerBase<T>::command_cart_pos,
                         this,ros::TransportHints().reliable().tcpNoDelay());

  // Variable init
  this->joint_msr_.resize(this->kdl_chain_.getNrOfJoints());
  q_dt_cmd_.resize(this->kdl_chain_.getNrOfJoints());
  q_cmd_.resize(this->kdl_chain_.getNrOfJoints());

  x_dt_des_ = KDL::Twist::Zero();
  x_des_.p.Zero();
  x_des_.M.Identity();
  x_.p.Zero();
  x_.M.Identity();
  x_dot_.p.Zero();
  x_dot_.M.Identity();

  return true;
}

/** \brief This is called from within the realtime thread just before the
 * first call to \ref update
 *
 * \param time The current time
 */
template<typename T>
void CartesianPositionControllerBase<T>::starting(const ros::Time& time){
  for(std::size_t i=0; i < this->joint_handles_.size(); i++) {
    q_dt_cmd_(i) = 0.0;
  }
  for(std::size_t i=0; i < this->joint_handles_.size(); i++) {
    q_cmd_(i) = 0.0;
  }
  x_des_.p.Zero();
  x_des_.M.Identity();
  x_dt_des_ = KDL::Twist::Zero();
  last_publish_time_ = time;
}

/*!
 * \brief Issues commands to the joint. Should be called at regular intervals
 */
template<typename T>
void CartesianPositionControllerBase<T>::update(const ros::Time& time,
                                         const ros::Duration& period) {

  // Get joint positions
  for(std::size_t i=0; i < this->joint_handles_.size(); i++)
  {
    this->joint_msr_.q(i)         = this->joint_handles_[i].getPosition();
    this->joint_msr_.qdot(i)      = this->joint_handles_[i].getVelocity();
  }
  fk_pos_solver_->JntToCart(this->joint_msr_.q, x_);

  // Compute inverse kinematics velocity solver
  // ik_vel_solver_->CartToJnt(this->joint_msr_.q, x_dt_des_, q_dt_cmd_);

  std::cout << "send_x_des_ " << "x: " << x_des_(0, 3) << "y: " << x_des_(1, 3)<< "z: " << x_des_(2, 3) << std::endl; 

  int check;
  check = ik_pos_solver_->CartToJnt(this->joint_msr_.q, x_des_, q_cmd_);
  if (check < 0)
  {
    std::cout << "shit fxxking happens " << check << std::endl; 

  }
  else
  {
    std::cout << "shit ends " << check << std::endl; 
    writePositionCommands(period);
  }

  // std::cout << "send" << q_cmd_(0) << std::endl; 
  
  // Forward kinematics
  // fk_vel_solver_->JntToCart(this->joint_msr_, x_dot_);
  // fk_pos_solver_->JntToCart(this->joint_msr_.q, x_);
  // std::cout << "send_x_ " << " x: " << x_(0, 3) << "y: " << x_(1, 3)<< "z: " << x_(2, 3) << std::endl; 

}

/*!
 * \brief Subscriber's callback: copies twist commands
 */
template<typename T>
void CartesianPositionControllerBase<T>::command_cart_pos(
                                     const geometry_msgs::PoseConstPtr &msg) {
    

    Vector3d x_text_vec_(msg->position.x, msg->position.y, msg->position.z);
    if (x_text_vec_.transpose() * x_text_vec_ > 0.01)
    {
      KDL::Vector x_dex_vec_(msg->position.x, - msg->position.y, msg->position.z);
      // so far only the position is controlled
      KDL::Frame x_mid_(x_.M, x_dex_vec_);
      x_des_ = x_mid_;
    }
}


/********************************************/
/**FUNCTIONS OF INSTANCES OF THE BASE CLASS**/
/********************************************/

void CartesianPositionController::writePositionCommands(
                                    const ros::Duration& period) {

    for(std::size_t i=0; i < this->joint_handles_.size(); i++) 
    {
      // double command = q_cmd_(i);
      // std::cout << command << std::endl;
      this->joint_handles_[i].setCommand(q_cmd_(i));
    }
}


/** \brief write the desired velocity command in the hardware interface input
 * for a VelocityJointInterface
 * \param period The duration of an update cycle
 */
void CartesianPositionControllerSim::writePositionCommands(
                                    const ros::Duration& period) {
  for(std::size_t i=0; i < this->joint_handles_.size(); i++) {
    this->joint_handles_[i].setCommand(this->joint_msr_.q(i)
                                    + q_dt_cmd_(i)*period.toSec());
  }
}

} // controller_interface namespace

// Register controllers with the PLUGINLIB_EXPORT_CLASS macro to enable dynamic
// loading with the controller manager
PLUGINLIB_EXPORT_CLASS(controller_interface::CartesianPositionController,
                       controller_interface::ControllerBase)
PLUGINLIB_EXPORT_CLASS(controller_interface::CartesianPositionControllerSim,
                       controller_interface::ControllerBase)