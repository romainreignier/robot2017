#pragma once

#include <array>
#include <memory>
#include <mutex>
#include <string>

#include <ros/ros.h>

#include <dynamic_reconfigure/server.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <realtime_tools/realtime_publisher.h>
#include <snd_msgs/Encoders.h>
#include <snd_msgs/Motors.h>
#include <snd_serial/SerialComm.h>
#include <snd_control/PidConfig.h>

namespace snd_control
{

class SndHardware : public hardware_interface::RobotHW
{
public:
  SndHardware();
  void read();
  void write();

private:
  void dynParamCb(PidConfig &_config, uint32_t _level);

  std::string m_name;
  ros::NodeHandle m_nh;
  dynamic_reconfigure::Server<PidConfig> m_dynParamServer;

  realtime_tools::RealtimePublisher<snd_msgs::Motors> m_motorsPub;
  realtime_tools::RealtimePublisher<snd_msgs::Encoders> m_encodersPub;

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

  std::unique_ptr<snd_serial::SerialComm> m_com;
  std::mutex m_comMutex;
};

} // namespace snd_control
