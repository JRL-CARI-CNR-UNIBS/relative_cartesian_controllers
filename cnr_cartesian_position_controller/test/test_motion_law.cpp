#include <ros/ros.h>
#include <cnr_cartesian_position_controller/cnr_cartesian_position_controller.h>


int main(int argc, char **argv)
{
  ros::init(argc, argv, "motion_law_test");
  ROS_INFO("start");


  ros::NodeHandle nh;
  ros::Publisher pub=nh.advertise<geometry_msgs::PoseStamped>("pose",1);
  ros::Publisher pub2=nh.advertise<geometry_msgs::PoseStamped>("stop_pose",1);
  ros::Publisher pub3=nh.advertise<geometry_msgs::TwistStamped>("twist",1);

  double max_cart_lin_vel=0.05;
  double max_cart_lin_acc=0.2;


  double max_cart_ang_vel=1;
  double max_cart_ang_acc=2;

  Eigen::Affine3d T_b_start;
  T_b_start.setIdentity();
  Eigen::Affine3d T_b_stop(Eigen::AngleAxisd(M_PI*0.5,Eigen::Vector3d::UnitY()));



  T_b_stop.translation()(2)=0.0;


  cnr::control::MotionLaw ml(max_cart_lin_vel,
                             max_cart_lin_acc,
                             max_cart_ang_vel,
                             max_cart_ang_acc,
                             T_b_stop,
                             T_b_start);

  double t_max=ml.getTmax();
  ROS_INFO("Tmax %f",t_max);

  Eigen::Affine3d T_base_actual;
  Eigen::Vector6d twist_in_base;

  geometry_msgs::PoseStamped pose;
  geometry_msgs::PoseStamped stop;
  geometry_msgs::TwistStamped twist;
  twist.header.frame_id="world";
  pose.header.frame_id="world";
  stop.header.frame_id="world";
  tf::poseEigenToMsg(T_b_stop,stop.pose);

  ros::Time t0=ros::Time::now();
  ros::Rate lp(100);
  while (ros::ok())
  {
    double t=(ros::Time::now()-t0).toSec();
    if (t>t_max*2)
      t0=ros::Time::now();
    ml.get(t,T_base_actual,twist_in_base);

    tf::poseEigenToMsg(T_base_actual,pose.pose);
    twist.twist.linear.x=twist_in_base(0);
    twist.twist.linear.y=twist_in_base(1);
    twist.twist.linear.z=twist_in_base(2);
    twist.twist.angular.x=twist_in_base(3);
    twist.twist.angular.y=twist_in_base(4);
    twist.twist.angular.z=twist_in_base(5);

    pose.header.stamp=ros::Time::now();
    stop.header.stamp=ros::Time::now();
    twist.header.stamp=ros::Time::now();

    pub.publish(pose);
    pub2.publish(stop);
    pub3.publish(twist);
    lp.sleep();
  }



}
