#pragma once

#include <memory>

#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <snd_serial/pidConfig.h>

#include "snd_serial/SerialComm.h"

class BoardComm
{
public:
  BoardComm();
  ~BoardComm();
  void run();
  void readStatus();

private:
  void cmdVelCb(const geometry_msgs::TwistConstPtr& _msg);
  void dynParamCb(snd_serial::pidConfig& _config, uint32_t _level);

  ros::NodeHandle m_nh;
  std::unique_ptr<snd_serial::SerialComm> m_com;
  ros::Publisher m_encodersPub;
  ros::Publisher m_starterPub;
  ros::Publisher m_odomPub;
  ros::Subscriber m_cmdVelSub;
  dynamic_reconfigure::Server<snd_serial::pidConfig> m_dynParamServer;

  double m_motorWheelSeparation;
  double m_motorWheelRadius;
  double m_encoderWheelSeparation;
  double m_encoderWheelRadius;
  double m_updateRate;
};
