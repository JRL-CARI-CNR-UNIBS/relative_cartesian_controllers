#pragma once

#include <cmath>
#include <Eigen/Core>

#include <tf/transform_listener.h>
#include <cnr_controller_interface/cnr_joint_command_controller_interface.h>
#include <cnr_hardware_interface/posveleff_command_interface.h>

#include <actionlib/server/action_server.h>
#include <relative_cartesian_controller_msgs/RelativeMoveAction.h>
#include <eigen_conversions/eigen_msg.h>
#include <rosdyn_core/frame_distance.h>
namespace ect = eigen_control_toolbox;

namespace cnr
{
namespace control
{


/**
 * @brief The CartesianPositionController class
 */
class CartesianPositionController:
    public cnr::control::JointCommandController<
                  hardware_interface::PosVelEffJointHandle, hardware_interface::PosVelEffJointInterface>
{
public:
  CartesianPositionController();
  bool doInit();
  bool doUpdate(const ros::Time& time, const ros::Duration& period);
  bool doStarting(const ros::Time& time);
  bool doStopping(const ros::Time& time);

protected:

  tf::TransformListener listener_;
  std::string tool_name_;
  std::shared_ptr<actionlib::ActionServer<relative_cartesian_controller_msgs::RelativeMoveAction>>             as_;
  std::shared_ptr<actionlib::ActionServer<relative_cartesian_controller_msgs::RelativeMoveAction>::GoalHandle> gh_;

  std::mutex mtx_;
  Eigen::Vector6d last_twist_of_setpoint_in_base_;

  double max_cart_lin_vel_;
  double max_cart_lin_acc_;
  double max_cart_ang_vel_;
  double max_cart_ang_acc_;
  double max_lin_dec_distance_;
  double max_ang_dec_distance_;

  double m_clik_gain;
  double linear_tolerance_=0.001;
  double angular_tolerance_=0.01;
  bool stop_thread_;
  bool check_actual_configuration_=true;
  bool singularity_=false;
  std::thread as_thread_;
  Eigen::Affine3d T_base_destination_;
  Eigen::Affine3d T_base_setpoint_;
  Eigen::Affine3d T_base_actual_;
  double target_linear_velocity_=0.001;
  double target_angular_velocity_=0.001;

  void actionGoalCallback   (actionlib::ActionServer<relative_cartesian_controller_msgs::RelativeMoveAction>::GoalHandle gh);
  void actionCancelCallback (actionlib::ActionServer<relative_cartesian_controller_msgs::RelativeMoveAction>::GoalHandle gh);
  void actionThreadFunction ( );

};


}  // end namespace control
}  // end namespace cnr
