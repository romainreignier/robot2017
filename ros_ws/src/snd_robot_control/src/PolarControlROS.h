#pragma once

#include "ros/ros.h"
#include <actionlib/server/simple_action_server.h>
#include <dynamic_reconfigure/server.h>
#include <nav_msgs/Odometry.h>
#include <snd_msgs/Motors.h>
#include <snd_msgs/MoveToAction.h>
#include <snd_robot_control/PolarControlConfig.h>

#include "libPolarControl/PolarControl.h"
#include "libPolarControl/Pose.h"

class PolarControlROS
{
public:
  PolarControlROS();

protected:
  // Subscribers Callbacks
  void odomCb(const nav_msgs::OdometryConstPtr& _msg);

  // Dynamic reconfigure callback
  void dynParamCb(const snd_robot_control::PolarControlConfig& _cfg, uint32_t);

  // Action servers callbacks
  void moveActionCb();
  void movePreemptCb();

  // Timer Callback
  void update(const ros::TimerEvent&);

  // Helpers
  Pose getCurrentPose() const;

protected:
  ros::NodeHandle m_nh;
  ros::Timer m_updateTimer;

  ros::Subscriber m_odomSub;
  ros::Publisher m_cmdPub;

  dynamic_reconfigure::Server<snd_robot_control::PolarControlConfig>
    m_dynParamServer;

  actionlib::SimpleActionServer<snd_msgs::MoveToAction> m_moveAs;

  nav_msgs::OdometryConstPtr m_lastOdom;

  PolarControl m_control;
};
