#include <boost/algorithm/string.hpp>
#include <ros/time.h>
#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Pose.h>
#include <pluginlib/class_list_macros.h>
#include <cnr_logger/cnr_logger_macros.h>

#include <cnr_cartesian_position_controller/cnr_cartesian_position_controller.h>


PLUGINLIB_EXPORT_CLASS(cnr::control::CartesianPositionController, controller_interface::ControllerBase)

namespace cnr
{
namespace control
{


/**
 * @brief CartesianPositionController::CartesianPositionController
 */
inline CartesianPositionController::CartesianPositionController()
{
}

/**
 * @brief CartesianPositionController::doInit
 * @return
 */
inline bool CartesianPositionController::doInit()
{
  CNR_TRACE_START(this->logger());

  //INIT PUB/SUB
  std::string setpoint_topic_name;
  this->setKinUpdatePeriod(this->m_sampling_period); // superimposing the fkin_update_period,
                                                     // we can then use chainCommand() sync and updated

  this->setPriority(this->QD_PRIORITY);

  this->setCommandVelocity(0.0*this->getVelocity()); //not needed, already superimposed in enterStarting()
  this->setCommandPosition(this->getPosition());

  if (!this->getControllerNh().getParam("max_cartesian_linear_speed",max_cart_lin_vel_))
  {
    CNR_INFO(this->logger(),this->getControllerNamespace()<<"/max_cartesian_linear_speed not defined, using 0.25 m/s");
    max_cart_lin_vel_=0.25;
  }

  if (!this->getControllerNh().getParam("max_cartesian_linear_acceleration",max_cart_lin_acc_))
  {
    CNR_INFO(this->logger(),this->getControllerNamespace()<<"/max_cartesian_linear_acceleration not defined, using 0.75 m/s^2");
    max_cart_lin_acc_=0.75;
  }

  if (!this->getControllerNh().getParam("cartesian_linear_tolerance",linear_tolerance_))
  {
    CNR_INFO(this->logger(),this->getControllerNamespace()<<"/cartesian_linear_tolerance not defined, using 0.75 m/s^2");
    linear_tolerance_=0.001;
  }
  if (!this->getControllerNh().getParam("cartesian_angular_tolerance",angular_tolerance_))
  {
    CNR_INFO(this->logger(),this->getControllerNamespace()<<"/cartesian_angular_tolerance not defined, using 0.75 m/s^2");
    angular_tolerance_=0.01;
  }

  if (!this->getControllerNh().getParam("max_cartesian_angular_speed",max_cart_ang_vel_))
  {
    CNR_INFO(this->logger(),this->getControllerNamespace()<<"/max_cartesian_angular_speed not defined, using 0.5 rad/s");
    max_cart_ang_vel_=0.5;
  }

  if (!this->getControllerNh().getParam("max_cartesian_angular_acceleration",max_cart_ang_acc_))
  {
    CNR_INFO(this->logger(),this->getControllerNamespace()<<"/max_cartesian_angular_acceleration not defined, using 1.5 rad/s^2");
    max_cart_ang_acc_=1.5;
  }

  if (!this->getControllerNh().getParam("clik_gain",m_clik_gain))
  {
    CNR_INFO(this->logger(),this->getControllerNamespace()<<"/clik_gain not defined, using 5.0");
    m_clik_gain=5.0;
  }

  if (!this->getControllerNh().getParam("check_actual_configuration",check_actual_configuration_))
  {
    CNR_INFO(this->logger(),this->getControllerNamespace()<<"/check_actual_configuration not defined, set true");
    check_actual_configuration_=true;
  }

  std::vector<std::string> overrides;
  if (!this->getControllerNh().getParam("overrides",overrides))
  {
    CNR_DEBUG(this->logger(),"overrides are not speficied for controllers. Using default");
    overrides.push_back("/speed_ovr");
    overrides.push_back("/safe_ovr_1");
    overrides.push_back("/safe_ovr_2");
  }

  CNR_TRACE(this->logger(),"subscribe override topics");

  for (const std::string& override_name: overrides)
  {
    auto cb=boost::bind(&CartesianPositionController::overrideCallback,this,_1,override_name);
    this->template add_subscriber<std_msgs::Int64>(override_name,1,cb,false);
    m_overrides.insert(std::pair<std::string,double>(override_name,1.0));
    CNR_DEBUG(this->logger(),"subscribe override = " << override_name);
  }
  m_global_override=1.0;

  CNR_TRACE(this->logger(),"create publisher");
  m_unscaled_pub_id = this->template add_publisher<sensor_msgs::JointState>("unscaled_joint_target",1);


  /* 0.5*max_acc*dec_time^2=dec_dist
   * max_acc*dec_time=max_vel -> dec_time=max_vel/max_acc
   * dec_dist=0.5*max_vel^2/max_dec
   */
  max_lin_dec_distance_=0.5*std::pow(max_cart_lin_vel_,2)/max_cart_lin_acc_;
  max_ang_dec_distance_=0.5*std::pow(max_cart_ang_vel_,2)/max_cart_ang_acc_;

  as_.reset(new actionlib::ActionServer<relative_cartesian_controller_msgs::RelativeMoveAction>(this->getControllerNh(), "relative_move",
                                                                      boost::bind(&CartesianPositionController::actionGoalCallback,    this,  _1),
                                                                      boost::bind(&CartesianPositionController::actionCancelCallback,  this,  _1),
                                                                      false));
  as_->start();

  tool_name_=this->chain().getLinksName().back();


  m_distance_pub = this->template add_publisher<sensor_msgs::JointState>("cartesian_distance",1000);

  tf::StampedTransform tf_base_tool;
  try
  {
    listener_.waitForTransform ( this->chain().getLinksName().front(),
                                 tool_name_,
                                 ros::Time(0),
                                 ros::Duration ( 10.0 ) );
    listener_.lookupTransform(this->chain().getLinksName().front(),
                              tool_name_,
                              ros::Time(0),
                              tf_base_tool
                              );
  }
  catch (std::exception& e)
  {
    CNR_ERROR(this->logger(),"unable to lookup tf tree, reason ="<<e.what());
    CNR_RETURN_FALSE(this->logger());
  }
  CNR_RETURN_TRUE(this->logger());
}


void CartesianPositionController::overrideCallback(const std_msgs::Int64ConstPtr& msg, const std::string& override_name)
{
  double ovr;
  if (msg->data>100)
    ovr=1.0;
  else if (msg->data<0)
    ovr=0.0;
  else
    ovr=msg->data*0.01;
  m_overrides.at(override_name)=ovr;
  double global_override=1;
  for (const std::pair<std::string,double>& p: m_overrides)
    global_override*=p.second;
  m_global_override=global_override;
}

/**
 * @brief CartesianPositionController::doStarting
 * @param time
 */
inline bool CartesianPositionController::doStarting(const ros::Time& /*time*/)
{
  CNR_TRACE_START(this->logger(),"Starting Controller");
  this->setCommandVelocity(0.0*this->getVelocity()); //not needed, already superimposed in enterStarting()
  this->setCommandPosition(this->getPosition());

  mtx_.lock();
  rosdyn::VectorXd q = this->getPosition();
  T_base_setpoint_=this->chainNonConst().getTransformation(q);
  T_base_destination_=T_base_setpoint_;
  T_base_actual_=T_base_setpoint_;
  mtx_.unlock();
  stop_thread_=false;
  last_twist_of_setpoint_in_base_ = Eigen::Vector6d::Zero();
  CNR_RETURN_TRUE(this->logger());
}

/**
 * @brief CartesianPositionController::stopping
 * @param time
 */
inline bool CartesianPositionController::doStopping(const ros::Time& /*time*/)
{
  CNR_TRACE_START(this->logger(),"Stopping Controller");

  stop_thread_=true;
  if (as_thread_.joinable())
  {
    as_thread_.join();
  }
  CNR_RETURN_TRUE(this->logger());
}

/**
 * @brief CartesianPositionController::doUpdate
 * @param time
 * @param period
 * @return
 */
inline bool CartesianPositionController::doUpdate(const ros::Time& /*time*/, const ros::Duration& period)
{
  CNR_TRACE_START_THROTTLE_DEFAULT(this->logger());
  std::stringstream report;


  mtx_.lock();
  rosdyn::VectorXd old_vel_sp = this->getCommandVelocity();
  rosdyn::VectorXd pos_sp = this->getCommandPosition();
  T_base_setpoint_=this->chainNonConst().getTransformation(pos_sp);
  rosdyn::VectorXd q = this->getPosition();
  T_base_actual_=this->chainNonConst().getTransformation(q);
  Eigen::Affine3d T_base_destination=T_base_destination_;
  mtx_.unlock();

  Eigen::Vector6d distance_in_base;
  rosdyn::getFrameDistance(T_base_destination,T_base_setpoint_,distance_in_base);
  double norm_lin_distance=distance_in_base.head(3).norm();
  double norm_ang_distance=distance_in_base.tail(3).norm();


  Eigen::Vector6d twist_of_setpoint_in_base;

  sensor_msgs::JointState dist_msg;
  dist_msg.name.resize(2);
  dist_msg.name.at(0)="translation";
  dist_msg.name.at(1)="rotation";
  dist_msg.position.resize(2,0);
  dist_msg.velocity.resize(2,0);
  dist_msg.effort.resize(2,0);
  dist_msg.header.stamp=ros::Time::now();

  Eigen::Vector6d tra_versor_in_base;
  double translation_vel;

  bool null_translation=false;
  bool null_rotation=false;
  bool tra_saturation=false;
  if (norm_lin_distance<1e-5)
  {
    tra_versor_in_base.setZero();
    translation_vel=0.0;
    null_translation=true;
  }
  else
  {
    tra_versor_in_base=distance_in_base/norm_lin_distance;
    translation_vel=tra_versor_in_base.head(3).dot(last_twist_of_setpoint_in_base_.head(3));

    double tra_dec_time=std::sqrt(2.0*norm_lin_distance/max_cart_lin_acc_);
    double dec_vel=max_cart_lin_acc_*tra_dec_time;
    double max_vel=std::min(target_linear_velocity_,max_cart_lin_vel_);
    double new_vel=std::min(max_vel,translation_vel+max_cart_lin_acc_*period.toSec());
    translation_vel=std::min(new_vel,dec_vel);
    tra_saturation= ((translation_vel==dec_vel) || (translation_vel==max_vel))&& (tra_dec_time>period.toSec()) ;

    dist_msg.position.at(0)=norm_lin_distance;
    dist_msg.velocity.at(0)=translation_vel;
    dist_msg.effort.at(0)=dec_vel;
  }

  Eigen::Vector6d rot_versor_in_base;
  double rotation_vel;
  bool rot_saturation=false;;
  if (norm_ang_distance<1e-5)
  {
    rot_versor_in_base.setZero();
    rotation_vel=0.0;
    rot_saturation=false;
    null_rotation=true;
  }
  else
  {
    rot_versor_in_base=distance_in_base/norm_ang_distance;
    rotation_vel=rot_versor_in_base.tail(3).dot(last_twist_of_setpoint_in_base_.tail(3));
    double rot_dec_time=std::sqrt(2.0*norm_ang_distance/max_cart_ang_acc_);
    double max_vel=std::min(target_angular_velocity_,max_cart_ang_vel_);
    double dec_vel=max_cart_ang_acc_*rot_dec_time;
    double new_vel=std::min(max_vel,rotation_vel+max_cart_ang_acc_*period.toSec());

    rotation_vel=std::min(new_vel,dec_vel);

    rot_saturation= ((rotation_vel==dec_vel) || (rotation_vel==max_vel)) && (rot_dec_time>period.toSec()) ;

    dist_msg.position.at(1)=norm_ang_distance;
    dist_msg.velocity.at(1)=rotation_vel;
    dist_msg.effort.at(1)=dec_vel;
  }

  if (null_rotation)  // pure translation
  {
    twist_of_setpoint_in_base=translation_vel*tra_versor_in_base;
    twist_of_setpoint_in_base.tail(3)/=1000.0;
  }
  else if (null_translation)  // pure rotation
  {
    twist_of_setpoint_in_base=rotation_vel*rot_versor_in_base;
    twist_of_setpoint_in_base.head(3)/=1000.0;
  }
  else if (tra_saturation) // compound movement, translation is saturated
  {
    twist_of_setpoint_in_base=translation_vel*tra_versor_in_base;
  }
  else if (rot_saturation) // compound movement, rotation is saturated
  {
    twist_of_setpoint_in_base=rotation_vel*rot_versor_in_base;
  }
  else // compound movement, no saturation, translation leading
  {
    twist_of_setpoint_in_base=translation_vel*tra_versor_in_base;
  }

  this->publish(m_distance_pub, dist_msg );

  Eigen::Matrix6Xd J_of_setpoint_in_base;
  J_of_setpoint_in_base=this->chainCommand().toolJacobian();  // CHECK IF CORRECT

  Eigen::JacobiSVD<Eigen::MatrixXd> svd(J_of_setpoint_in_base, Eigen::ComputeThinU | Eigen::ComputeThinV);
  auto sv = svd.singularValues();
  CNR_WARN_COND_THROTTLE(this->logger(),
                          (sv(sv.rows()-1)==0) || (sv(0)/sv(sv.rows()-1) > 1e2), 2, "SINGULARITY POINT" );

  rosdyn::VectorXd vel_sp = svd.solve(twist_of_setpoint_in_base);


  sensor_msgs::JointStatePtr unscaled_js_msg(new sensor_msgs::JointState());
  unscaled_js_msg->name = this->jointNames();
  unscaled_js_msg->position.resize(this->nAx());
  unscaled_js_msg->velocity.resize(this->nAx());
  unscaled_js_msg->effort.resize(this->nAx(),0);
  for (size_t iax=0;iax<this->nAx();iax++)
  {
    unscaled_js_msg->position.at(iax) = q(iax);
    unscaled_js_msg->velocity.at(iax) = vel_sp(iax);
  }
  unscaled_js_msg->header.stamp  = ros::Time::now();
  this->publish(m_unscaled_pub_id,unscaled_js_msg);

  vel_sp*=m_global_override;

  if(rosdyn::saturateSpeed(this->chainNonConst(),vel_sp,old_vel_sp,
                           this->getCommandPosition(),period.toSec(), 1.0, true, &report)) // CHECK!
  {
    CNR_DEBUG_THROTTLE(this->logger(), 2.0, "\n" << report.str() );
  }

  Eigen::Vector6d twist_of_t_in_bcommand=J_of_setpoint_in_base*vel_sp;
  Eigen::Vector6d versor=twist_of_setpoint_in_base.normalized();
  Eigen::Vector6d parallel_twist=twist_of_t_in_bcommand.dot(versor)*versor;
  Eigen::Vector6d perpendicular_twist=twist_of_t_in_bcommand -parallel_twist;

  if (perpendicular_twist.norm()>1e-6)
  {
    vel_sp*=1e-6/perpendicular_twist.norm();
    CNR_WARN_THROTTLE(this->logger(),1,"saturating velocity, direction error (perpendicular norm = " << perpendicular_twist.norm() <<  ") due to singularity and joint limits");
    CNR_DEBUG_THROTTLE(this->logger(),1,
                      "twist_of_t_in_b         = " << twist_of_setpoint_in_base.transpose() << std::endl <<
                      "twist_of_t_in_bcommand = " << twist_of_t_in_bcommand.transpose() << std::endl <<
                      "parallel_twist velocity = " << parallel_twist.transpose() << std::endl <<
                      "perpedicular velocity   = " << perpendicular_twist.transpose()
                      );
  }

  last_twist_of_setpoint_in_base_=J_of_setpoint_in_base*vel_sp;

  if(rosdyn::saturateSpeed(this->chainNonConst(),vel_sp,old_vel_sp,
                              this->getCommandPosition(),period.toSec(), 1.0, true, &report)) // CHECK!
  {
    CNR_DEBUG_THROTTLE(this->logger(), 2.0, "\n" << report.str() );
  }


  pos_sp = this->getCommandPosition() + vel_sp * period.toSec();

  if(rosdyn::saturatePosition(this->chainNonConst(),pos_sp, &report))
  {
    CNR_DEBUG_THROTTLE(this->logger(), 2.0, "\n" << report.str() );
  }

  last_twist_of_setpoint_in_base_=J_of_setpoint_in_base*vel_sp;
  this->setCommandPosition( pos_sp );
  this->setCommandVelocity( vel_sp );

  CNR_RETURN_TRUE_THROTTLE_DEFAULT(this->logger());
}

/**
 * @brief CartesianPositionController::actionGoalCallback
 * @param gh
 */
void CartesianPositionController::actionGoalCallback(actionlib::ActionServer< relative_cartesian_controller_msgs::RelativeMoveAction>::GoalHandle gh)
{
  try
  {
    if (gh_)
    {
      CNR_INFO(this->logger(),"preepmt previous goal");
      stop_thread_=true;
    }
    auto goal = gh.getGoal();
    singularity_=false;
    std::shared_ptr<actionlib::ActionServer<relative_cartesian_controller_msgs::RelativeMoveAction>::GoalHandle> current_gh;

    current_gh.reset(new actionlib::ActionServer<relative_cartesian_controller_msgs::RelativeMoveAction>::GoalHandle(gh));
    gh_ = current_gh;


    Eigen::Affine3d T_setpoint_destination;
    tf::poseMsgToEigen(goal->relative_pose.pose,T_setpoint_destination);
    if (goal->relative_pose.header.frame_id=="TOOL" || goal->relative_pose.header.frame_id==tool_name_)
    {
      tf::poseMsgToEigen(goal->relative_pose.pose,T_setpoint_destination);
    }
    else
    {
      tf::StampedTransform tf_tool_frame;
      try
      {
        listener_.waitForTransform ( tool_name_,
                                     goal->relative_pose.header.frame_id,
                                     ros::Time(0),
                                     ros::Duration ( 10.0 ) );
        listener_.lookupTransform(tool_name_,
                                  goal->relative_pose.header.frame_id,
                                  ros::Time(0),
                                  tf_tool_frame);
      }
      catch (std::exception& e)
      {
        CNR_ERROR(this->logger(),
                  "unable to find a transformation tool("<<
                  tool_name_<<
                  ") <== destination("<<
                  goal->relative_pose.header.frame_id<<
                  "). reason = " <<
                  e.what());
        T_setpoint_destination.setIdentity();
        relative_cartesian_controller_msgs::RelativeMoveResult result;
        result.error_code   = relative_cartesian_controller_msgs::RelativeMoveResult::INVALID_FRAME;
        result.error_string = "invalid frame id";
        gh_->setRejected(result);
        return;
      }

      Eigen::Affine3d T_tool_frame;
      tf::poseTFToEigen(tf_tool_frame,T_tool_frame);

      Eigen::Affine3d T_setpoint_destination_in_frame;
      tf::poseMsgToEigen(goal->relative_pose.pose,T_setpoint_destination_in_frame);
      Eigen::AngleAxisd aa_in_frame(T_setpoint_destination_in_frame.linear());
      Eigen::Vector3d axis_in_frame=aa_in_frame.axis();
      double angle=aa_in_frame.angle();
      Eigen::Vector3d axis_in_t=T_tool_frame.linear()*axis_in_frame;

      Eigen::AngleAxisd aa_in_setpoint(angle,axis_in_t);
      T_setpoint_destination=aa_in_setpoint;

      Eigen::Vector3d p_destination_in_frame=T_setpoint_destination_in_frame.translation();
      Eigen::Vector3d p_destination_in_setpoint=T_tool_frame.linear()*p_destination_in_frame;
      T_setpoint_destination.translation()=p_destination_in_setpoint;
    }

    target_linear_velocity_=goal->target_linear_velocity;
    if (target_linear_velocity_<=0)
    {
      CNR_ERROR(this->logger(),"target linear velocity should be positive");
      T_setpoint_destination.setIdentity();
      relative_cartesian_controller_msgs::RelativeMoveResult result;
      result.error_code   = relative_cartesian_controller_msgs::RelativeMoveResult::INVALID_TARGET_VELOCITY;
      result.error_string = "target linear velocity should be positive";
      gh_->setRejected(result);
      return;
    }
    target_angular_velocity_=goal->target_angular_velocity;
    if (target_angular_velocity_<=0)
    {
      CNR_ERROR(this->logger(),"target angular velocity should be positive");
      T_setpoint_destination.setIdentity();
      relative_cartesian_controller_msgs::RelativeMoveResult result;
      result.error_code   = relative_cartesian_controller_msgs::RelativeMoveResult::INVALID_TARGET_VELOCITY;
      result.error_string = "target angular velocity should be positive";
      gh_->setRejected(result);
      return;
    }
    CNR_INFO(this->logger(),"[ "<<this->getControllerNamespace()<<" ] New Goal Received, action start!");
    gh_->setAccepted();

    mtx_.lock();
    T_base_destination_=T_base_setpoint_*T_setpoint_destination;
    mtx_.unlock();

    if (as_thread_.joinable())
    {
      as_thread_.join();
    }

    as_thread_    = std::thread(&CartesianPositionController::actionThreadFunction,this);
  }
  catch( std::exception& e )
  {
    CNR_ERROR(this->logger(),"Exception. what: " << e.what() );
    relative_cartesian_controller_msgs::RelativeMoveResult result;
    result.error_code   = relative_cartesian_controller_msgs::RelativeMoveResult::UNKNOWN;
    result.error_string = std::string("exception: ")+e.what();
    gh_->setAborted(result);
  }
  catch( ... )
  {
    CNR_ERROR(this->logger(),"Generalized Exception.");
    relative_cartesian_controller_msgs::RelativeMoveResult result;
    result.error_code   = relative_cartesian_controller_msgs::RelativeMoveResult::UNKNOWN;
    result.error_string = "goal exception";
    gh_->setAborted(result);
  }

}

void CartesianPositionController::actionCancelCallback(actionlib::ActionServer< relative_cartesian_controller_msgs::RelativeMoveAction >::GoalHandle /*gh*/)
{
  if (gh_)
  {
    gh_->setCanceled();
    stop_thread_ = true;
    if (as_thread_.joinable())
    {
      as_thread_.join();
    }
    gh_.reset();
  }
  else
  {
    CNR_WARN(this->logger(),"Triggered the Cancel of the Action but none Goal is active.");
  }
}

void CartesianPositionController::actionThreadFunction()
{
  ros::WallRate lp(100);

  ros::Time leaving_timer;

  while (this->getControllerNh().ok())
  {
    lp.sleep();
    if (!gh_)
    {
      CNR_ERROR(this->logger(),"Goal handle is not initialized");
      break;
    }
    relative_cartesian_controller_msgs::RelativeMoveFeedback fb;
    relative_cartesian_controller_msgs::RelativeMoveResult result;
    mtx_.lock();
    Eigen::Affine3d T_base_setpoint=T_base_setpoint_;
    Eigen::Affine3d T_base_actual=T_base_actual_;
    Eigen::Affine3d T_b_destination=T_base_destination_;
    mtx_.unlock();

    Eigen::Vector6d distance;
    if (check_actual_configuration_)
      rosdyn::getFrameDistance(T_b_destination,T_base_actual,distance);
    else
      rosdyn::getFrameDistance(T_b_destination,T_base_setpoint,distance);

    if (distance.head(3).norm()<linear_tolerance_ && distance.tail(3).norm()<angular_tolerance_)
    {
      result.error_code   = relative_cartesian_controller_msgs::RelativeMoveResult::SUCCESS;
      result.error_string = "finished";
      gh_->setSucceeded(result);
      break;
    }

    if( stop_thread_ )
    {
      CNR_ERROR(this->logger(),"Triggered an external stop. Break the action loop.");
      result.error_code   = relative_cartesian_controller_msgs::RelativeMoveResult::CANCELLED;
      result.error_string = "Cancelled";
      mtx_.lock();
      T_base_destination_=T_base_setpoint_;
      mtx_.unlock();
      gh_->setAborted(result);
      break;
    }

    if (singularity_)
    {
      CNR_ERROR(this->logger(),"Singularity");
      result.error_code   = relative_cartesian_controller_msgs::RelativeMoveResult::SINGULARITY;
      result.error_string = "Singularity";
      gh_->setAborted(result);
      mtx_.lock();
      T_base_destination_=T_base_setpoint_;
      mtx_.unlock();
      break;
    }

    gh_->publishFeedback(fb);
  }
  gh_.reset();
}


}  // end namespace control
}  // end namespace cnr
