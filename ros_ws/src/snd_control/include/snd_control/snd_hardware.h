#pragma once

#include <array>
#include <atomic>
#include <mutex>
#include <string>

#include <ros/ros.h>

#include <dynamic_reconfigure/server.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <realtime_tools/realtime_publisher.h>
#include <snd_control/PidConfig.h>
#include <snd_msgs/Encoders.h>
#include <snd_msgs/Motors.h>
#include <snd_msgs/Pid.h>

namespace snd_control
{

class SndHardwareRos : public hardware_interface::RobotHW
{
public:
  SndHardwareRos();
  void read();
  void write();

private:
  void encodersCb(const snd_msgs::EncodersConstPtr& _msg);
  void dynParamCb(PidConfig& _config, uint32_t _level);

  std::string m_name;
  ros::NodeHandle m_nh;
  dynamic_reconfigure::Server<PidConfig> m_dynParamServer;

  ros::Subscriber m_encodersSub;

  realtime_tools::RealtimePublisher<snd_msgs::Motors> m_motorsPub;
  realtime_tools::RealtimePublisher<snd_msgs::Pid> m_leftMotorPidPub;
  realtime_tools::RealtimePublisher<snd_msgs::Pid> m_rightMotorPidPub;

  hardware_interface::JointStateInterface m_jointStateInterface;
  hardware_interface::VelocityJointInterface m_velocityJointInterface;

  struct Joint
  {
    std::string name;
    double position;
    double velocity;
    double velocityCommand;
    double effort;

    Joint() : position(0), velocity(0), velocityCommand(0), effort(0) {}
  };
  std::array<Joint, 2> m_joints;
  double m_ticksToRad;
  snd_msgs::EncodersConstPtr m_encodersMsg;
  std::mutex m_encodersMsgMutex;
};

} // namespace snd_control
