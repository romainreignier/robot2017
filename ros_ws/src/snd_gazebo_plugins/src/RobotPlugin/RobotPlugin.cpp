#include "RobotPlugin.h"

#include <algorithm>
#include <assert.h>

#include <gazebo/math/gzmath.hh>
#include <sdf/sdf.hh>

#include <ros/ros.h>

namespace gazebo
{

enum
{
  RIGHT,
  LEFT,
};

// Load the controller
void GazeboRosDiffDriveLowLevel::Load(physics::ModelPtr _parent,
                                      sdf::ElementPtr _sdf)
{
  this->parent = _parent;
  gazebo_ros_ = GazeboRosPtr(new GazeboRos(_parent, _sdf, "DiffDriveLowLevel"));
  // Make sure the ROS node for Gazebo has already been initialized
  gazebo_ros_->isInitialized();

  gazebo_ros_->getParameter<std::string>(
    command_topic_, "commandTopic", "/motors");
  gazebo_ros_->getParameter<std::string>(
    odometry_topic_, "odometryTopic", "odom");
  gazebo_ros_->getParameter<std::string>(
    odometry_frame_, "odometryFrame", "odom");
  gazebo_ros_->getParameter<std::string>(
    robot_base_frame_, "robotBaseFrame", "base_footprint");
  gazebo_ros_->getParameterBoolean(publishWheelTF_, "publishWheelTF", false);
  gazebo_ros_->getParameterBoolean(publishTF_, "publishTf", false);
  gazebo_ros_->getParameterBoolean(
    publishWheelEncoders_, "publishWheelEncoders", false);
  gazebo_ros_->getParameterBoolean(
    publishWheelJointState_, "publishWheelJointState", false);
  gazebo_ros_->getParameter<double>(wheel_separation_, "wheelSeparation", 0.18);
  gazebo_ros_->getParameter<double>(wheel_diameter_, "wheelDiameter", 0.060);
  gazebo_ros_->getParameter<unsigned>(
    encoder_resolution_, "encoderResolution", 2400);
  gazebo_ros_->getParameter<double>(wheel_torque, "wheelTorque", 5.0);
  gazebo_ros_->getParameter<double>(update_rate_, "updateRate", 100.0);
  std::map<std::string, OdomSource> odomOptions;
  odomOptions["encoder"] = ENCODER;
  odomOptions["world"] = WORLD;
  gazebo_ros_->getParameter<OdomSource>(
    odom_source_, "odometrySource", odomOptions, WORLD);

  gzmsg << "wheel diameter: " << wheel_diameter_ << '\n';
  gzmsg << "wheels separation: " << wheel_separation_ << '\n';

  joints_.resize(2);
  joints_[LEFT] = gazebo_ros_->getJoint(parent, "leftJoint", "left_joint");
  joints_[RIGHT] = gazebo_ros_->getJoint(parent, "rightJoint", "right_joint");
  joints_[LEFT]->SetParam("fmax", 0, wheel_torque);
  joints_[RIGHT]->SetParam("fmax", 0, wheel_torque);

  // Initialize update rate stuff
  if(this->update_rate_ > 0.0)
    this->update_period_ = 1.0 / this->update_rate_;
  else
    this->update_period_ = 0.0;
  last_update_time_ = parent->GetWorld()->GetSimTime();

  vel_cmd_[LEFT] = 0;
  vel_cmd_[RIGHT] = 0;
  wheel_absolute_angle_[LEFT] = 0;
  wheel_absolute_angle_[RIGHT] = 0;
  last_wheel_angle_[LEFT] = 0;
  last_wheel_angle_[RIGHT] = 0;
  encoder_counter_[LEFT] = 0;
  encoder_counter_[RIGHT] = 0;
  alive_ = true;

  if(this->publishWheelJointState_)
  {
    joint_state_publisher_ =
      gazebo_ros_->node()->advertise<sensor_msgs::JointState>("joint_states",
                                                              1000);
    ROS_INFO_NAMED(
      "diff_drive", "%s: Advertise joint_states", gazebo_ros_->info());
  }

  if(this->publishWheelEncoders_)
  {
    encoders_publisher_ =
      gazebo_ros_->node()->advertise<snd_msgs::Encoders>("encoders", 10);
    ROS_INFO_NAMED("diff_drive", "%s: Advertise encoders", gazebo_ros_->info());
  }

  transform_broadcaster_ =
    boost::shared_ptr<tf::TransformBroadcaster>(new tf::TransformBroadcaster());

  // ROS: Subscribe to the velocity command topic
  ROS_INFO_NAMED("diff_drive",
                 "%s: Try to subscribe to %s",
                 gazebo_ros_->info(),
                 command_topic_.c_str());

  ros::SubscribeOptions so = ros::SubscribeOptions::create<snd_msgs::Motors>(
    command_topic_,
    1,
    boost::bind(&GazeboRosDiffDriveLowLevel::cmdVelCallback, this, _1),
    ros::VoidPtr(),
    &queue_);

  cmd_vel_subscriber_ = gazebo_ros_->node()->subscribe(so);
  ROS_INFO_NAMED("diff_drive",
                 "%s: Subscribe to %s",
                 gazebo_ros_->info(),
                 command_topic_.c_str());

  if(this->publishTF_)
  {
    odometry_publisher_ =
      gazebo_ros_->node()->advertise<nav_msgs::Odometry>(odometry_topic_, 1);
    ROS_INFO_NAMED("diff_drive",
                   "%s: Advertise odom on %s ",
                   gazebo_ros_->info(),
                   odometry_topic_.c_str());
  }

  // start custom queue for diff drive
  this->callback_queue_thread_ =
    boost::thread(boost::bind(&GazeboRosDiffDriveLowLevel::QueueThread, this));

  // listen to the update event (broadcast every simulation iteration)
  this->update_connection_ = event::Events::ConnectWorldUpdateBegin(
    boost::bind(&GazeboRosDiffDriveLowLevel::UpdateChild, this));
}

void GazeboRosDiffDriveLowLevel::Reset()
{
  last_update_time_ = parent->GetWorld()->GetSimTime();
  pose_encoder_.x = 0;
  pose_encoder_.y = 0;
  pose_encoder_.theta = 0;
  vel_cmd_[LEFT] = 0;
  vel_cmd_[RIGHT] = 0;
  wheel_absolute_angle_[LEFT] = 0;
  wheel_absolute_angle_[RIGHT] = 0;
  last_wheel_angle_[LEFT] = 0;
  last_wheel_angle_[RIGHT] = 0;
  encoder_counter_[LEFT] = 0;
  encoder_counter_[RIGHT] = 0;
  joints_[LEFT]->SetParam("fmax", 0, wheel_torque);
  joints_[RIGHT]->SetParam("fmax", 0, wheel_torque);
}

void GazeboRosDiffDriveLowLevel::publishWheelJointState()
{
  ros::Time current_time = ros::Time::now();

  joint_state_.header.stamp = current_time;
  joint_state_.name.resize(joints_.size());
  joint_state_.position.resize(joints_.size());

  for(size_t i = 0; i < 2; i++)
  {
    physics::JointPtr joint = joints_[i];
    math::Angle angle = joint->GetAngle(0);
    joint_state_.name[i] = joint->GetName();
    joint_state_.position[i] = angle.Radian();
  }
  joint_state_publisher_.publish(joint_state_);
}

void GazeboRosDiffDriveLowLevel::publishFakeEncoders()
{
  for(int i = 0; i < 2; i++)
  {
    encoder_counter_[i] = static_cast<int64_t>(
      (wheel_absolute_angle_[i] * encoder_resolution_) / (2 * M_PI));
  }
  snd_msgs::Encoders msg;
  msg.left = encoder_counter_[LEFT];
  msg.right = encoder_counter_[RIGHT];
  encoders_publisher_.publish(msg);
}

void GazeboRosDiffDriveLowLevel::publishWheelTF()
{
  ros::Time current_time = ros::Time::now();
  for(size_t i = 0; i < 2; i++)
  {

    std::string wheel_frame =
      gazebo_ros_->resolveTF(joints_[i]->GetChild()->GetName());
    std::string wheel_parent_frame =
      gazebo_ros_->resolveTF(joints_[i]->GetParent()->GetName());

    math::Pose poseWheel = joints_[i]->GetChild()->GetRelativePose();

    tf::Quaternion qt(
      poseWheel.rot.x, poseWheel.rot.y, poseWheel.rot.z, poseWheel.rot.w);
    tf::Vector3 vt(poseWheel.pos.x, poseWheel.pos.y, poseWheel.pos.z);

    tf::Transform tfWheel(qt, vt);
    transform_broadcaster_->sendTransform(tf::StampedTransform(
      tfWheel, current_time, wheel_parent_frame, wheel_frame));
  }
}

// Update the controller
void GazeboRosDiffDriveLowLevel::UpdateChild()
{
  if(odom_source_ == ENCODER) UpdateOdometryEncoder();

  // Update fake encoders
  const double angle_l = joints_[LEFT]->GetAngle(0).Radian();
  const double angle_r = joints_[RIGHT]->GetAngle(0).Radian();
  const double delta_angle_l = angle_l - last_wheel_angle_[LEFT];
  const double delta_angle_r = angle_r - last_wheel_angle_[RIGHT];
  wheel_absolute_angle_[LEFT] += delta_angle_l;
  wheel_absolute_angle_[RIGHT] += delta_angle_r;
  last_wheel_angle_[LEFT] = angle_l;
  last_wheel_angle_[RIGHT] = angle_r;

  common::Time current_time = parent->GetWorld()->GetSimTime();
  double seconds_since_last_update =
    (current_time - last_update_time_).Double();

  if(seconds_since_last_update > update_period_)
  {
    if(this->publishTF_) publishOdometry(seconds_since_last_update);
    if(publishWheelTF_) publishWheelTF();
    if(publishWheelJointState_) publishWheelJointState();
    if(publishWheelEncoders_) publishFakeEncoders();

    for(size_t i = 0; i < 2; i++)
    {
      joints_[i]->SetParam("vel", 0, vel_cmd_[i]);
    }

    last_update_time_ += common::Time(update_period_);
  }
}

// Finalize the controller
void GazeboRosDiffDriveLowLevel::FiniChild()
{
  alive_ = false;
  queue_.clear();
  queue_.disable();
  gazebo_ros_->node()->shutdown();
  callback_queue_thread_.join();
}

void GazeboRosDiffDriveLowLevel::cmdVelCallback(
  const snd_msgs::Motors::ConstPtr& cmd_msg)
{
  boost::mutex::scoped_lock scoped_lock(lock);
  vel_cmd_[LEFT] = static_cast<double>(cmd_msg->left);
  vel_cmd_[RIGHT] = static_cast<double>(cmd_msg->right);
}

void GazeboRosDiffDriveLowLevel::QueueThread()
{
  static const double timeout = 0.01;

  while(alive_ && gazebo_ros_->node()->ok())
  {
    queue_.callAvailable(ros::WallDuration(timeout));
  }
}

void GazeboRosDiffDriveLowLevel::UpdateOdometryEncoder()
{
  double vl = joints_[LEFT]->GetVelocity(0);
  double vr = joints_[RIGHT]->GetVelocity(0);
  common::Time current_time = parent->GetWorld()->GetSimTime();
  double seconds_since_last_update =
    (current_time - last_odom_update_).Double();
  last_odom_update_ = current_time;

  double b = wheel_separation_;

  // Book: Sigwart 2011 Autonompus Mobile Robots page:337
  double sl = vl * (wheel_diameter_ / 2.0) * seconds_since_last_update;
  double sr = vr * (wheel_diameter_ / 2.0) * seconds_since_last_update;
  double ssum = sl + sr;

  double sdiff = sr - sl;

  double dx = (ssum) / 2.0 * cos(pose_encoder_.theta + (sdiff) / (2.0 * b));
  double dy = (ssum) / 2.0 * sin(pose_encoder_.theta + (sdiff) / (2.0 * b));
  double dtheta = (sdiff) / b;

  pose_encoder_.x += dx;
  pose_encoder_.y += dy;
  pose_encoder_.theta += dtheta;

  double w = dtheta / seconds_since_last_update;
  double v = sqrt(dx * dx + dy * dy) / seconds_since_last_update;

  tf::Quaternion qt;
  tf::Vector3 vt;
  qt.setRPY(0, 0, pose_encoder_.theta);
  vt = tf::Vector3(pose_encoder_.x, pose_encoder_.y, 0);

  odom_.pose.pose.position.x = vt.x();
  odom_.pose.pose.position.y = vt.y();
  odom_.pose.pose.position.z = vt.z();

  odom_.pose.pose.orientation.x = qt.x();
  odom_.pose.pose.orientation.y = qt.y();
  odom_.pose.pose.orientation.z = qt.z();
  odom_.pose.pose.orientation.w = qt.w();

  odom_.twist.twist.angular.z = w;
  odom_.twist.twist.linear.x = dx / seconds_since_last_update;
  odom_.twist.twist.linear.y = dy / seconds_since_last_update;
}

void GazeboRosDiffDriveLowLevel::publishOdometry(double step_time)
{

  ros::Time current_time = ros::Time::now();
  std::string odom_frame = gazebo_ros_->resolveTF(odometry_frame_);
  std::string base_footprint_frame = gazebo_ros_->resolveTF(robot_base_frame_);

  tf::Quaternion qt;
  tf::Vector3 vt;

  if(odom_source_ == ENCODER)
  {
    // getting data form encoder integration
    qt = tf::Quaternion(odom_.pose.pose.orientation.x,
                        odom_.pose.pose.orientation.y,
                        odom_.pose.pose.orientation.z,
                        odom_.pose.pose.orientation.w);
    vt = tf::Vector3(odom_.pose.pose.position.x,
                     odom_.pose.pose.position.y,
                     odom_.pose.pose.position.z);
  }
  if(odom_source_ == WORLD)
  {
    // getting data form gazebo world
    math::Pose pose = parent->GetWorldPose();
    qt = tf::Quaternion(pose.rot.x, pose.rot.y, pose.rot.z, pose.rot.w);
    vt = tf::Vector3(pose.pos.x, pose.pos.y, pose.pos.z);

    odom_.pose.pose.position.x = vt.x();
    odom_.pose.pose.position.y = vt.y();
    odom_.pose.pose.position.z = vt.z();

    odom_.pose.pose.orientation.x = qt.x();
    odom_.pose.pose.orientation.y = qt.y();
    odom_.pose.pose.orientation.z = qt.z();
    odom_.pose.pose.orientation.w = qt.w();

    // get velocity in /odom frame
    math::Vector3 linear;
    linear = parent->GetWorldLinearVel();
    odom_.twist.twist.angular.z = parent->GetWorldAngularVel().z;

    // convert velocity to child_frame_id (aka base_footprint)
    float yaw = pose.rot.GetYaw();
    odom_.twist.twist.linear.x = cosf(yaw) * linear.x + sinf(yaw) * linear.y;
    odom_.twist.twist.linear.y = cosf(yaw) * linear.y - sinf(yaw) * linear.x;
  }

  tf::Transform base_footprint_to_odom(qt, vt);
  transform_broadcaster_->sendTransform(tf::StampedTransform(
    base_footprint_to_odom, current_time, odom_frame, base_footprint_frame));

  // set covariance
  odom_.pose.covariance[0] = 0.00001;
  odom_.pose.covariance[7] = 0.00001;
  odom_.pose.covariance[14] = 1000000000000.0;
  odom_.pose.covariance[21] = 1000000000000.0;
  odom_.pose.covariance[28] = 1000000000000.0;
  odom_.pose.covariance[35] = 0.001;

  // set header
  odom_.header.stamp = current_time;
  odom_.header.frame_id = odom_frame;
  odom_.child_frame_id = base_footprint_frame;

  odometry_publisher_.publish(odom_);
}

GZ_REGISTER_MODEL_PLUGIN(GazeboRosDiffDriveLowLevel)
}
