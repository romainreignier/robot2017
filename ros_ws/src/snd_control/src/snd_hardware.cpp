#include <rosparam_shortcuts/rosparam_shortcuts.h>

#include "snd_control/snd_hardware.h"

namespace snd_control
{

SndHardware::SndHardware() : m_name("snd_hardware")
{
  // Retrieve parameters
  int encoderResolution;
  std::string serialPort;
  std::size_t serialBaudrate;
  std::size_t serialTimeout;

  namespace rps = rosparam_shortcuts;
  std::size_t error = 0;
  ros::NodeHandle rpNh(m_nh, m_name);
  error += !rps::get(m_name, rpNh, "left_wheel_joint_name", m_joints[0].name);
  error += !rps::get(m_name, rpNh, "right_wheel_joint_name", m_joints[1].name);
  error += !rps::get(m_name, rpNh, "encoder_resolution", encoderResolution);
  error += !rps::get(m_name, rpNh, "serial_port", serialPort);
  error += !rps::get(m_name, rpNh, "serial_baudrate", serialBaudrate);
  error += !rps::get(m_name, rpNh, "serial_timeout", serialTimeout);
  rps::shutdownIfError(m_name, error);

  // Create the communication
  m_com = std::make_unique<snd_serial::SerialComm>(
    serialPort, serialBaudrate, serialTimeout);
  if(m_com->isOpen())
  {
    ROS_INFO_STREAM("Serial port " << serialPort << " successfully openned");
  }
  else
  {
    ROS_FATAL_STREAM("Could not open serial port " << serialPort);
    m_nh.shutdown();
  }

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
  m_motorsPub.init(m_nh, "motors", 1);
  m_encodersPub.init(m_nh, "encoders", 1);

  // Compute the convertion factor only once
  m_ticksToRad = 2.0 * M_PI / encoderResolution;

  // Create the Dynamic Reconfigure server
  m_dynParamServer.setCallback(
    boost::bind(&SndHardware::dynParamCb, this, _1, _2));
}

void SndHardware::read()
{
  // Send request to get status
  snd_proto::SerialRequest req;
  snd_proto::EmptyMsg* empty = req.mutable_getstatus();
  {
    std::lock_guard<std::mutex> lock{m_comMutex};
    m_com->sendMsg(req);
  }
  try
  {
    snd_proto::SerialResponse resp = m_com->readIncomingMsg();
    switch(resp.type_case())
    {
    case snd_proto::SerialResponse::kStatus:
    {
      // Save the encoder position
      m_joints[0].position = resp.status().encoders().left() * m_ticksToRad;
      m_joints[1].position = resp.status().encoders().right() * m_ticksToRad;

      // Publish the encoders positions for debug
      if(m_encodersPub.trylock())
      {
        m_encodersPub.msg_.header.stamp = ros::Time::now();
        m_encodersPub.msg_.header.frame_id = "base_link";
        m_encodersPub.msg_.left = resp.status().encoders().left();
        m_encodersPub.msg_.right = resp.status().encoders().right();
        m_encodersPub.unlockAndPublish();
      }
    }
    break;
    default:
      ROS_WARN_STREAM("Message reveived is not a status message (type: "
                      << resp.type_case()
                      << ").");
    }
  }
  catch(const std::runtime_error& _e)
  {
    ROS_ERROR_STREAM("Error while reading status: " << _e.what());
  }
}

void SndHardware::write()
{
  // Send a request to change motor velocity
  snd_proto::SerialRequest req;
  snd_proto::Speed* speed = req.mutable_setmotorsspeed();
  const float leftCommandTicks = m_joints[0].velocityCommand / m_ticksToRad;
  const float rightCommandTicks = m_joints[1].velocityCommand / m_ticksToRad;
  speed->set_left(leftCommandTicks);
  speed->set_right(rightCommandTicks);
  try
  {
    std::lock_guard<std::mutex> lock{m_comMutex};
    m_com->sendMsg(req);

    // Publish motors commands for debug
    if(m_motorsPub.trylock())
    {
      m_motorsPub.msg_.header.stamp = ros::Time::now();
      m_motorsPub.msg_.header.frame_id = "base_link";
      m_motorsPub.msg_.left = leftCommandTicks;
      m_motorsPub.msg_.right = rightCommandTicks;
      m_motorsPub.unlockAndPublish();
    }
  }
  catch(const std::runtime_error& _e)
  {
    ROS_ERROR_STREAM("Error while sending motor velocity: " << _e.what());
  }
}

void SndHardware::dynParamCb(PidConfig& _config, uint32_t _level)
{
  try
  {
    // Left PID values modified
    if(_level & 0x1)
    {
      ROS_INFO_STREAM("Changing Left Speed Pid to P: " << _config.left_speed_p
                                                       << " I: "
                                                       << _config.left_speed_i
                                                       << " D: "
                                                       << _config.left_speed_d);
      snd_proto::SerialRequest req;
      snd_proto::PidTunings* pid = req.mutable_setpidspeedleft();
      pid->set_p(_config.left_speed_p);
      pid->set_i(_config.left_speed_i);
      pid->set_d(_config.left_speed_d);
      {
        std::lock_guard<std::mutex> lock{m_comMutex};
        m_com->sendMsg(req);
      }
    }
    // Right PID values modified
    if(_level & 0x2)
    {
      ROS_INFO_STREAM(
        "Changing Reft Speed Pid to P: " << _config.right_speed_p << " I: "
                                         << _config.right_speed_i
                                         << " D: "
                                         << _config.right_speed_d);
      snd_proto::SerialRequest req;
      snd_proto::PidTunings* pid = req.mutable_setpidspeedright();
      pid->set_p(_config.right_speed_p);
      pid->set_i(_config.right_speed_i);
      pid->set_d(_config.right_speed_d);
      {
        std::lock_guard<std::mutex> lock{m_comMutex};
        m_com->sendMsg(req);
      }
    }
  }
  catch(const std::runtime_error& _e)
  {
    ROS_ERROR_STREAM("Error while sending Pid config: " << _e.what());
  }
}

} // snd_control namespace
