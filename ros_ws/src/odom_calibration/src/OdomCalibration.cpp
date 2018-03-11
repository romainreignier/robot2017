#include "OdomCalibration.h"

#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>

OdomCalibration::OdomCalibration() : m_nh{"~"}
{
  m_leftTicksSub = m_nh.subscribe("/tickg", 10, &OdomCalibration::leftTicksCb, this);
  m_rightTicksSub = m_nh.subscribe("/tickd", 10, &OdomCalibration::rightTicksCb, this);

  m_odomPub = m_nh.advertise<nav_msgs::Odometry>("/odom", 10);

  ROS_INFO_STREAM("Default wheel separation: " << m_odom.wheelSeparationMM << " mm");
  if(m_nh.getParam("wheel_separation", m_odom.wheelSeparationMM))
  {
    ROS_INFO_STREAM("New wheel separation: " << m_odom.wheelSeparationMM << " mm");
  }

  ROS_INFO_STREAM("Default left radius: " << m_odom.leftWheelRadius << " mm");
  if(m_nh.getParam("left_radius", m_odom.leftWheelRadius))
  {
    ROS_INFO_STREAM("New left radius: " << m_odom.leftWheelRadius << " mm");
  }

  ROS_INFO_STREAM("Default right radius: " << m_odom.rightWheelRadius << " mm");
  if(m_nh.getParam("right_radius", m_odom.rightWheelRadius))
  {
    ROS_INFO_STREAM("New right radius: " << m_odom.rightWheelRadius << " mm");
  }

  m_odom.reset();

  m_lastTime = ros::Time::now();

  ROS_INFO("Ready");
}

void OdomCalibration::leftTicksCb(const std_msgs::Float32& msg)
{
  m_currentLeftTicks = msg.data;
}

void OdomCalibration::rightTicksCb(const std_msgs::Float32& msg)
{
  m_currentRightTicks = msg.data;

  ros::Time now = ros::Time::now();
  if((now - m_lastTime).toSec() < 0)
  {
    // Reset time
    ROS_WARN("Reset because dt < 0");
    m_odom.reset();
    m_currentLeftTicks = 0;
    m_currentRightTicks = 0;

    m_lastTime = now;
    return;
  }
  // Not very accurate, not sure leftTicks was updated...
  const Pose pose = m_odom.computeOdom(m_currentLeftTicks, m_currentRightTicks);
  publishOdom(pose, now);
}

void OdomCalibration::publishOdom(const Pose& pose, const ros::Time& now)
{
  // Broadcast TF
  const geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromYaw(pose.theta);
  geometry_msgs::TransformStamped odom_trans;
  odom_trans.header.stamp = now;
  odom_trans.header.frame_id = "odom";
  odom_trans.child_frame_id = "base_link";
  odom_trans.transform.translation.x = pose.x / 1000.0;
  odom_trans.transform.translation.y = pose.y / 1000.0;
  odom_trans.transform.rotation = quat;

  m_tfBroadcaster.sendTransform(odom_trans);

  // Publish odom
  nav_msgs::Odometry msg;
  msg.header.stamp = now;
  msg.header.frame_id = "odom";
  msg.child_frame_id = "base_link";
  msg.pose.pose.position.x = pose.x / 1000.0;
  msg.pose.pose.position.y = pose.y / 1000.0;
  msg.pose.pose.orientation = quat;

  m_odomPub.publish(msg);

  m_lastTime = now;
}
