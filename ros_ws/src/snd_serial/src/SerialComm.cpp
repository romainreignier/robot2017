
#include <string>
#include <iostream>

#include <ros/console.h>
#include <std_msgs/Bool.h>

#include <snd_serial/Encoders.h>
#include <snd_serial/pidConfig.h>

#include "SerialComm.h"

using namespace std;

SerialComm::SerialComm()
    : m_nh{"~"}, m_motorWheelSeparation{0.188}, m_motorWheelRadius{0.35},
      m_encoderWheelSeparation{0.104}, m_encoderWheelRadius{0.25},
      m_updateRate{10}
{
  // Check Protobuf version
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  // Handle the global parameters
  if(!ros::param::get("motor_wheel_separation", m_motorWheelSeparation))
  {
    ROS_WARN_STREAM("No 'motor_wheel_separation' param found. Default: "
                    << m_motorWheelSeparation);
  }
  if(!ros::param::get("motor_wheel_radius", m_motorWheelRadius))
  {
    ROS_WARN_STREAM(
        "No 'motor_wheel_radius' param found. Default: " << m_motorWheelRadius);
  }
  if(!ros::param::get("encoder_wheel_separation", m_encoderWheelSeparation))
  {
    ROS_WARN_STREAM("No 'encoder_wheel_separation' param found. Default: "
                    << m_encoderWheelSeparation);
  }
  if(!ros::param::get("encoder_wheel_radius", m_encoderWheelRadius))
  {
    ROS_WARN_STREAM("No 'encoder_wheel_radius' param found. Default: "
                    << m_encoderWheelRadius);
  }

  // Handle the private parameters
  if(!m_nh.getParam("update_rate", m_updateRate))
  {
    ROS_WARN_STREAM("No 'update_rate' param found. Default: "
                    << m_updateRate << " Hz");
  }
  string port("/dev/ttyACM0");
  if(!m_nh.getParam("port", port))
  {
    ROS_WARN_STREAM("No 'port' param found. Default: " << port);
  }
  int baudrate(115200);
  if(!m_nh.getParam("baudrate", baudrate))
  {
    ROS_WARN_STREAM("No 'baudrate' param found. Default: " << baudrate);
  }
  ROS_INFO_STREAM("Openning serial port " << port << " at " << baudrate
                                          << " bauds.");
  m_serial = make_unique<serial::Serial>(
      port, baudrate, serial::Timeout::simpleTimeout(200));
  if(m_serial->isOpen())
  {
    ROS_INFO_STREAM("Serial port " << port << " successfully openned");
  }
  else
  {
    ROS_FATAL_STREAM("Could not open serial port " << port);
    m_nh.shutdown();
  }

  // Create the publishers
  m_encodersPub = m_nh.advertise<snd_serial::Encoders>("encoders", 1);
  m_starterPub = m_nh.advertise<std_msgs::Bool>("starter", 1);

  // Create the subscribers
  m_cmdVelSub = m_nh.subscribe("/cmd_vel", 1, &SerialComm::cmdVelCb, this);

  // Create the Dynamic Reconfigure server
  m_dynParamServer.setCallback(
    boost::bind(&SerialComm::dynParamCb, this, _1, _2));
}

SerialComm::~SerialComm()
{
  google::protobuf::ShutdownProtobufLibrary();
  m_serial->close();
}

void SerialComm::run()
{
  ros::Rate rate(m_updateRate);
  while(ros::ok())
  {
    readStatus();
    ros::spinOnce();
    rate.sleep();
  }
}

void printBuffer(const uint8_t* _buffer, size_t _length)
{
    for(size_t i = 0 ; i < _length ; ++i)
    {
      cout << "0x" << std::hex << static_cast<uint16_t>(_buffer[i]) << " ";
    }
    cout << std::dec << endl;
}

void SerialComm::readIncomingMsg()
{
  // Read the first byte to get the length of the message
  uint8_t length;
  size_t readBytes = m_serial->read(reinterpret_cast<uint8_t*>(&length), 1);
  if(readBytes < 1)
  {
    ROS_WARN("Timeout while retrieving the message length.");
  }
  else
  {
    // Read the message of length bytes
    string msg;
    readBytes = m_serial->read(msg, length);
    if(readBytes < 1)
    {
      ROS_WARN("Tiemout while retrieving the message.");
    }
    else
    {
      snd_msgs::SerialResponse resp;
      if(!resp.ParseFromString(msg))
      {
        ROS_ERROR("Failed to parse the protobuf message.");
        cout << "Received length: " << static_cast<uint16_t>(length) << endl;
        cout << "Read bytes: " << msg.length() << endl;
        cout << "0x" << std::hex << static_cast<uint16_t>(length) << " ";
        printBuffer(reinterpret_cast<const uint8_t*>(msg.data()), msg.length());
        return;
      }
      switch(resp.type_case())
      {
      case snd_msgs::SerialResponse::kLog:
        switch(resp.log().level())
        {
        case snd_msgs::Log::DEBUG: ROS_DEBUG_STREAM(resp.log().text()); break;
        case snd_msgs::Log::INFO: ROS_INFO_STREAM(resp.log().text()); break;
        case snd_msgs::Log::WARN: ROS_WARN_STREAM(resp.log().text()); break;
        case snd_msgs::Log::ERROR: ROS_ERROR_STREAM(resp.log().text()); break;
        case snd_msgs::Log::FATAL: ROS_FATAL_STREAM(resp.log().text()); break;
        }
        break;
      case snd_msgs::SerialResponse::kEncoders:
        ROS_INFO_STREAM("Encoders left: " << resp.encoders().left()
                                          << " right: "
                                          << resp.encoders().right());
        {
          snd_serial::Encoders msg;
          msg.header.stamp = ros::Time::now();
          msg.left = resp.encoders().left();
          msg.right = resp.encoders().right();
          m_encodersPub.publish(msg);
        }

        break;
      case snd_msgs::SerialResponse::kPose:
        ROS_INFO_STREAM("Robot in position x:" << resp.pose().x() << " y: "
                                               << resp.pose().y()
                                               << " theta: "
                                               << resp.pose().th());
        break;
      case snd_msgs::SerialResponse::kSpeed:
        ROS_INFO_STREAM("Motors speed left:" << resp.speed().left()
                                             << " right: "
                                             << resp.speed().right());
        break;
      case snd_msgs::SerialResponse::kIsStarterSet:
        if(resp.isstarterset())
        {
          ROS_INFO_STREAM("Starter set");
        }
        else
        {
          ROS_INFO_STREAM("Starter not set");
        }
        {
          std_msgs::Bool msg;
          msg.data = resp.isstarterset();
          m_starterPub.publish(msg);
        }
        break;
      case snd_msgs::SerialResponse::kStatus:
        // ROS_INFO_STREAM("Status message:");
        // ROS_INFO_STREAM("Pose: x: " << resp.status().pose().x() << " y: " <<
        // resp.status().pose().y() << " th: " << resp.status().pose().th());
        // ROS_INFO_STREAM("Speed: left: " << resp.status().speed().left() << "
        // right: " << resp.status().speed().right());
        // ROS_INFO_STREAM("Starter: " << (resp.status().starter() ? "set" :
        // "not
        // set"));
        // ROS_INFO_STREAM("eStop: " << (resp.status().estop() ? "set" : "not
        // set"));
        // ROS_INFO_STREAM("IrBarrier: left: " << (resp.status().ir().left() ?
        // "active" : "not active") << " center: " <<
        // (resp.status().ir().center()
        // ? "active" : "not active") << " right: " <<
        // (resp.status().ir().right()
        // ? "active" : "not active"));
        // ROS_INFO_STREAM("Encoders: left: " << resp.status().encoders().left()
        // << " right: " << resp.status().encoders().right());
        {
          snd_serial::Encoders msg;
          msg.header.stamp = ros::Time::now();
          msg.left = resp.status().encoders().left();
          msg.right = resp.status().encoders().right();
          m_encodersPub.publish(msg);
        }
        break;

      default:
        ROS_WARN_STREAM("Message type: " << resp.type_case()
                                         << " not handled.");
      }
    }
  }
}

int SerialComm::sendMsg(const snd_msgs::SerialRequest& _req)
{
  // Serialize the protobuf
  string outString;
  if(!_req.SerializeToString(&outString))
  {
    ROS_ERROR("Failed to serialize the message.");
    return -1;
  }
  // make sure the message length can be encoded on 1 byte
  assert(outString.length() < 0xff);
  uint8_t length = outString.length();
  size_t bytesWritten = m_serial->write(&length, 1);
  bytesWritten += m_serial->write(outString);
  // A bit of waste of time be to be sure to separate each packet
  m_serial->waitByteTimes(bytesWritten);
  return static_cast<int>(bytesWritten);
}

void SerialComm::cmdVelCb(const geometry_msgs::TwistConstPtr& _msg)
{
  double velLinearLeft =
      _msg->linear.x - _msg->angular.z * m_motorWheelSeparation / 2.0;
  double velLinearRight =
      _msg->linear.x + _msg->angular.z * m_motorWheelSeparation / 2.0;

  double velAngularLeft = velLinearLeft / m_motorWheelRadius;
  double velAngularRight = velLinearRight / m_motorWheelRadius;

  ROS_INFO_STREAM("Twist x: " << _msg->linear.x << " z: " << _msg->angular.z
                  << " converted into left: " << velAngularLeft
                  << " right: " << velAngularRight);

  snd_msgs::SerialRequest req;
  snd_msgs::Speed* speed = req.mutable_setmotorsspeed();
  speed->set_left(static_cast<float>(velAngularLeft));
  speed->set_right(static_cast<float>(velAngularRight));
  sendMsg(req);
}

void SerialComm::readStatus()
{
  snd_msgs::SerialRequest req;
  snd_msgs::EmptyMsg* empty = req.mutable_getstatus();
  sendMsg(req);
  readIncomingMsg();
}

void SerialComm::dynParamCb(snd_serial::pidConfig& _config, uint32_t _level)
{
  ROS_INFO_STREAM("Level: " << _level);
  // Left PID values modified
  if(_level & 0x1)
  {
    ROS_INFO_STREAM("Changing Left Speed Pid to P: " << _config.left_speed_p
                                                     << " I: "
                                                     << _config.left_speed_i
                                                     << " D: "
                                                     << _config.left_speed_d);
    snd_msgs::SerialRequest req;
    snd_msgs::PidTunings* pid = req.mutable_setpidspeedleft();
    pid->set_p(_config.left_speed_p);
    pid->set_i(_config.left_speed_i);
    pid->set_d(_config.left_speed_d);
    sendMsg(req);
  }
  // Right PID values modified
  if(_level & 0x2)
  {
    ROS_INFO_STREAM("Changing Reft Speed Pid to P: " << _config.right_speed_p
                                                     << " I: "
                                                     << _config.right_speed_i
                                                     << " D: "
                                                     << _config.right_speed_d);
    snd_msgs::SerialRequest req;
    snd_msgs::PidTunings* pid = req.mutable_setpidspeedright();
    pid->set_p(_config.right_speed_p);
    pid->set_i(_config.right_speed_i);
    pid->set_d(_config.right_speed_d);
    sendMsg(req);
  }
}

// vim: sw=2 ts=2 et
