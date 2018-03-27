#pragma once

#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <tf/transform_broadcaster.h>

#include "Odometry.h"

class OdomCalibration
{
public:
  OdomCalibration();

private:
  void leftTicksCb(const std_msgs::Float32& msg);
  void rightTicksCb(const std_msgs::Float32& msg);

  void publishOdom(const Pose& pose, const ros::Time& now);

  ros::NodeHandle m_nh;
  ros::Subscriber m_leftTicksSub;
  ros::Subscriber m_rightTicksSub;
  ros::Publisher m_odomPub;
  tf::TransformBroadcaster m_tfBroadcaster;
  ros::Time m_lastTime;

  float m_currentLeftTicks = 0;
  float m_currentRightTicks = 0;
  Odometry m_odom;
};
