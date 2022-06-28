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


class MotionLaw
{
public:
  MotionLaw(double max_cart_lin_vel,
            double max_cart_lin_acc,
            double max_cart_ang_vel,
            double max_cart_ang_acc,
            const Eigen::Affine3d& T_base_destination,
            const Eigen::Affine3d& T_base_start):
    max_cart_lin_vel_(max_cart_lin_vel),
    max_cart_lin_acc_(max_cart_lin_acc),
    max_cart_ang_vel_(max_cart_ang_vel),
    max_cart_ang_acc_(max_cart_ang_acc)
  {
    T_base_destination_=T_base_destination;
    T_base_start_=T_base_start;

    Eigen::Vector6d distance_in_base;
    rosdyn::getFrameDistance(T_base_start_,T_base_destination_,distance_in_base);
    norm_lin_distance_=distance_in_base.head(3).norm();
    norm_ang_distance_=distance_in_base.tail(3).norm();


    Eigen::AngleAxisd aa_in_start(T_base_start_.linear().inverse() * T_base_destination_.linear());

    tra_axis_in_start_=T_base_start_.linear().inverse()*(T_base_destination_.translation() - T_base_start_.translation()).normalized();
    rot_axis_in_start_=aa_in_start.axis();
    double t_max_lin;
    double t1_lin;
    double t2_lin;
    double t_max_ang;
    double t1_ang;
    double t2_ang;

    computeMinTime(norm_lin_distance_,
                   max_cart_lin_vel_,
                   max_cart_lin_acc_,
                   t1_lin,
                   t2_lin,
                   t_max_lin);
    computeMinTime(norm_ang_distance_,
                   max_cart_ang_vel_,
                   max_cart_ang_acc_,
                   t1_ang,
                   t2_ang,
                   t_max_ang);


    if (t_max_lin>t_max_ang)
    {
      t_max_=t_max_lin;
      t1_=t1_lin;
      t2_=t2_lin;
    }
    else {
      t_max_=t_max_ang;
      t1_=t1_ang;
      t2_=t2_ang;
    }

    computeVelAcc(norm_lin_distance_,
                  t1_,
                  t2_,
                  lin_vel_,
                  lin_acc_);
    computeVelAcc(norm_ang_distance_,
                  t1_,
                  t2_,
                  ang_vel_,
                  ang_acc_);
  }

  double getTmax()
  {
    return t_max_;
  }

  void get(const double& t,
           Eigen::Affine3d& T_b_actual,
           Eigen::Vector6d& twist_in_b)
  {
    double pos,vel,acc;
    compute(t,
            t1_,
            t2_,
            t_max_,
            norm_lin_distance_,
            lin_vel_,
            lin_acc_,
            pos,
            vel,
            acc);

    Eigen::Vector3d tra=tra_axis_in_start_*pos;
    Eigen::Vector6d twist_in_a;
    twist_in_a.head(3)=  tra_axis_in_start_*vel;
    compute(t,
            t1_,
            t2_,
            t_max_,
            norm_ang_distance_,
            ang_vel_,
            ang_acc_,
            pos,
            vel,
            acc);

    Eigen::AngleAxisd aa(pos,rot_axis_in_start_);
    Eigen::Affine3d T_start_actual(aa);
    T_start_actual.translation()=tra;
    T_b_actual=T_base_start_*T_start_actual;

    twist_in_a.tail(3)=rot_axis_in_start_*vel;

    twist_in_b=rosdyn::spatialRotation(twist_in_a,T_b_actual.linear().inverse());

  }

protected:
  double max_cart_lin_vel_;
  double max_cart_lin_acc_;
  double max_cart_ang_vel_;
  double max_cart_ang_acc_;
  double max_lin_dec_distance_;
  double max_ang_dec_distance_;

  Eigen::Affine3d T_base_destination_;
  Eigen::Affine3d T_base_start_;
  Eigen::Affine3d T_base_actual_;
  Eigen::Vector3d rot_axis_in_start_;
  Eigen::Vector3d tra_axis_in_start_;

  double t_max_,t1_,t2_;
  double lin_vel_, lin_acc_,ang_vel_,ang_acc_;
  double norm_lin_distance_;
  double norm_ang_distance_;


  double period_;

  void computeMinTime(const double& distance,
                      const double& vel,
                      const double& acc,
                      double& t1,
                      double& t2,
                      double& tmax)
  {
    double t_acc=vel/acc; // time to reach max speed
    double s=t_acc*vel;   // distance to reach max speed and slow down immediately (threshold between triangular and trapeziodal motion lows)

    double t_const_vel;
    if (distance>s) // trapeziodal
    {
      t_const_vel=(distance-s)/vel;
    }
    else // triangular
    {
      t_acc=std::sqrt(distance/acc);
      t_const_vel=0.0;
    }
    t1=t_acc;
    t2=t_acc+t_const_vel;
    tmax=t2+t_acc;
  }

  void computeVelAcc(const double& distance,
                     const double& t1,
                     const double& t2,
                     double& v,
                     double& a)
  {
    a=distance/(t1*t1+(t2-t1)*t1);
    v=a*t1;
  }

  void compute(const double& t,
               const double& t1,
               const double& t2,
               const double& tmax,
               const double& distance,
               const double& v,
               const double& a,
               double& pos,
               double& vel,
               double& acc)
  {
    if (t<0.0)
    {
      pos=0.0;
      vel=0.0;
      acc=0.0;
    }
    else if (t<t1)
    {
      pos=0.5*a*t*t;
      vel=a*t;
      acc=a;
    }
    else if (t<t2)
    {
      double s1=0.5*a*t1*t1;
      pos=s1+v*(t-t1);
      vel=v;
      acc=0.0;
    }
    else if (t<tmax)
    {
      double s1=0.5*a*t1*t1;
      double s2=s1+v*(t2-t1);
      double dt=t-t2;
      pos=s2+v*dt-0.5*a*dt*dt;
      vel=v-a*dt;
      acc=-a;
    }
    else
    {
      pos=distance;
      vel=0.0;
      acc=0.0;
    }
  }
};



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
  rosdyn::VectorXd last_pos_sp_;

  double max_cart_lin_vel_;
  double max_cart_lin_acc_;
  double max_cart_ang_vel_;
  double max_cart_ang_acc_;

  size_t m_scaled_pub_id;
  size_t m_unscaled_pub_id;

  std::map<std::string,double> m_overrides;
  std::vector<ros::Subscriber> m_override_topic;
  double m_global_override;



  std::shared_ptr<MotionLaw> ml_;
  double scaled_time_=0.0;
  int trials_=0;

  double linear_tolerance_=0.001;
  double angular_tolerance_=0.01;
  bool stop_thread_;
  bool check_actual_configuration_=true;
  std::thread as_thread_;
  Eigen::Affine3d T_base_destination_;
  Eigen::Affine3d T_base_setpoint_;
  Eigen::Affine3d T_base_actual_;
  double target_linear_velocity_=0.001;
  double target_angular_velocity_=0.001;


  void actionGoalCallback   (actionlib::ActionServer<relative_cartesian_controller_msgs::RelativeMoveAction>::GoalHandle gh);
  void actionCancelCallback (actionlib::ActionServer<relative_cartesian_controller_msgs::RelativeMoveAction>::GoalHandle gh);
  void actionThreadFunction ( );


  void overrideCallback(const std_msgs::Int64ConstPtr& msg, const std::string& override_name);

};



}  // end namespace control
}  // end namespace cnr
