#pragma once

// Gazebo
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_plugins/gazebo_ros_utils.h>

// ROS
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <snd_msgs/Encoders.h>
#include <snd_msgs/Motors.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

// Custom Callback Queue
#include <ros/advertise_options.h>
#include <ros/callback_queue.h>

// Boost
#include <boost/bind.hpp>
#include <boost/thread.hpp>

namespace gazebo
{

class Joint;
class Entity;

class GazeboRosDiffDriveLowLevel : public ModelPlugin
{

  enum OdomSource
  {
    ENCODER = 0,
    WORLD = 1,
  };

public:
  GazeboRosDiffDriveLowLevel() = default;
  ~GazeboRosDiffDriveLowLevel() = default;
  void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
  void Reset();

protected:
  virtual void UpdateChild();
  virtual void FiniChild();

private:
  void publishOdometry(double step_time);
  void getWheelVelocities();
  void publishWheelTF();
  void publishWheelJointState();
  void publishFakeEncoders();
  void UpdateOdometryEncoder();

  GazeboRosPtr gazebo_ros_;
  physics::ModelPtr parent;
  event::ConnectionPtr update_connection_;

  double wheel_separation_;
  double wheel_diameter_;
  double wheel_torque;
  double wheel_speed_[2];
  unsigned encoder_resolution_;

  std::vector<physics::JointPtr> joints_;

  // ROS STUFF
  ros::Publisher odometry_publisher_;
  ros::Publisher encoders_publisher_;
  ros::Subscriber cmd_vel_subscriber_;
  boost::shared_ptr<tf::TransformBroadcaster> transform_broadcaster_;
  sensor_msgs::JointState joint_state_;
  ros::Publisher joint_state_publisher_;
  nav_msgs::Odometry odom_;
  std::string tf_prefix_;

  boost::mutex lock;

  std::string robot_namespace_;
  std::string command_topic_;
  std::string odometry_topic_;
  std::string odometry_frame_;
  std::string robot_base_frame_;

  // Custom Callback Queue
  ros::CallbackQueue queue_;
  boost::thread callback_queue_thread_;
  void QueueThread();

  // DiffDrive stuff
  void cmdVelCallback(const snd_msgs::Motors::ConstPtr& cmd_msg);

  double vel_cmd_[2];
  int64_t encoder_counter_[2];
  double wheel_absolute_angle_[2];
  double last_wheel_angle_[2];
  bool alive_;

  // Update Rate
  double update_rate_;
  double update_period_;
  common::Time last_update_time_;

  OdomSource odom_source_;
  geometry_msgs::Pose2D pose_encoder_;
  common::Time last_odom_update_;

  // Flags
  bool publishTF_;
  bool publishWheelTF_;
  bool publishWheelJointState_;
  bool publishWheelEncoders_;
};

} // namespace gazebo
