#include <rosparam_shortcuts/rosparam_shortcuts.h>

#include "snd_control/snd_hardware.h"

namespace snd_control
{

SndHardwareRos::SndHardwareRos() : m_name("snd_hardware")
{
  // Retrieve parameters
  int encoderResolution;

  namespace rps = rosparam_shortcuts;
  std::size_t error = 0;
  ros::NodeHandle rpNh(m_nh, m_name);
  error += !rps::get(m_name, rpNh, "left_wheel_joint_name", m_joints[0].name);
  error += !rps::get(m_name, rpNh, "right_wheel_joint_name", m_joints[1].name);
  error += !rps::get(m_name, rpNh, "encoder_resolution", encoderResolution);
  rps::shutdownIfError(m_name, error);

  for(auto& joint : m_joints)
  {
    hardware_interface::JointStateHandle jointStateHandle(
      joint.name, &joint.position, &joint.velocity, &joint.effort);
    m_jointStateInterface.registerHandle(jointStateHandle);

    hardware_interface::JointHandle jointHandle(jointStateHandle,
                                                &joint.velocityCommand);
    m_velocityJointInterface.registerHandle(jointHandle);
  }
  registerInterface(&m_jointStateInterface);
  registerInterface(&m_velocityJointInterface);

  // realtime publishers so initialization differs
  m_motorsPub.init(m_nh, "/motors_speed", 1);
  m_leftMotorPidPub.init(m_nh, "/left_motor_pid", 1);
  m_rightMotorPidPub.init(m_nh, "/right_motor_pid", 1);

  // Subscriber
  m_encodersSub =
    m_nh.subscribe("/encoders", 1, &SndHardwareRos::encodersCb, this);

  // Compute the convertion factor only once
  m_ticksToRad = 2.0 * M_PI / encoderResolution;

  // Create the Dynamic Reconfigure server
  m_dynParamServer.setCallback(
    boost::bind(&SndHardwareRos::dynParamCb, this, _1, _2));
}

void SndHardwareRos::read()
{
  std::unique_lock<std::mutex> lock(m_encodersMsgMutex, std::try_to_lock);
  if(m_encodersMsg && lock.owns_lock())
  {
    // Save the encoder position
    m_joints[0].position = m_encodersMsg->left_pos * m_ticksToRad;
    m_joints[1].position = m_encodersMsg->right_pos * m_ticksToRad;
  }
}

void SndHardwareRos::write()
{
  const float leftCommandTicks = m_joints[0].velocityCommand / m_ticksToRad;
  const float rightCommandTicks = m_joints[1].velocityCommand / m_ticksToRad;

  // Publish motors commands
  if(m_motorsPub.trylock())
  {
    m_motorsPub.msg_.header.stamp = ros::Time::now();
    m_motorsPub.msg_.header.frame_id = "base_link";
    m_motorsPub.msg_.left = leftCommandTicks;
    m_motorsPub.msg_.right = rightCommandTicks;
    m_motorsPub.unlockAndPublish();
  }
}

void SndHardwareRos::encodersCb(const snd_msgs::EncodersConstPtr& _msg)
{
  std::lock_guard<std::mutex> lock{m_encodersMsgMutex};
  m_encodersMsg = _msg;
}

void SndHardwareRos::dynParamCb(PidConfig& _config, uint32_t _level)
{
  // Left PID values modified
  if(_level & 0x1)
  {
    ROS_INFO_STREAM("Changing Left Speed Pid to P: " << _config.left_speed_p
                                                     << " I: "
                                                     << _config.left_speed_i
                                                     << " D: "
                                                     << _config.left_speed_d);
    if(m_leftMotorPidPub.trylock())
    {
      m_leftMotorPidPub.msg_.p = _config.left_speed_p;
      m_leftMotorPidPub.msg_.i = _config.left_speed_i;
      m_leftMotorPidPub.msg_.d = _config.left_speed_d;
      m_leftMotorPidPub.unlockAndPublish();
    }
  }
  // Right PID values modified
  if(_level & 0x2)
  {
    ROS_INFO_STREAM("Changing Reft Speed Pid to P: " << _config.right_speed_p
                                                     << " I: "
                                                     << _config.right_speed_i
                                                     << " D: "
                                                     << _config.right_speed_d);
    if(m_rightMotorPidPub.trylock())
    {
      m_rightMotorPidPub.msg_.p = _config.right_speed_p;
      m_rightMotorPidPub.msg_.i = _config.right_speed_i;
      m_rightMotorPidPub.msg_.d = _config.right_speed_d;
      m_rightMotorPidPub.unlockAndPublish();
    }
  }
}

} // snd_control namespace
