
#include <iostream>
#include <string>

#include <ros/console.h>
#include <std_msgs/Bool.h>

#include <snd_msgs/Encoders.h>
#include <snd_serial/pidConfig.h>

#include "BoardComm.h"
#include "CommMsgs.pb.h"

using namespace std;

BoardComm::BoardComm()
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
    ROS_WARN_STREAM("No 'update_rate' param found. Default: " << m_updateRate
                                                              << " Hz");
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
  m_com = make_unique<snd_serial::SerialComm>(port, baudrate, 200);
  if(m_com->isOpen())
  {
    ROS_INFO_STREAM("Serial port " << port << " successfully openned");
  }
  else
  {
    ROS_FATAL_STREAM("Could not open serial port " << port);
    m_nh.shutdown();
  }

  // Create the publishers
  m_encodersPub = m_nh.advertise<snd_msgs::Encoders>("encoders", 1);
  m_starterPub = m_nh.advertise<std_msgs::Bool>("starter", 1);

  // Create the subscribers
  m_cmdVelSub = m_nh.subscribe("/cmd_vel", 1, &BoardComm::cmdVelCb, this);

  // Create the Dynamic Reconfigure server
  m_dynParamServer.setCallback(
    boost::bind(&BoardComm::dynParamCb, this, _1, _2));
}

BoardComm::~BoardComm()
{
  google::protobuf::ShutdownProtobufLibrary();
}

void BoardComm::run()
{
  ros::Rate rate(m_updateRate);
  while(ros::ok())
  {
    readStatus();
    ros::spinOnce();
    rate.sleep();
  }
}

void BoardComm::cmdVelCb(const geometry_msgs::TwistConstPtr& _msg)
{
  double velLinearLeft =
    _msg->linear.x - _msg->angular.z * m_motorWheelSeparation / 2.0;
  double velLinearRight =
    _msg->linear.x + _msg->angular.z * m_motorWheelSeparation / 2.0;

  double velAngularLeft = velLinearLeft / m_motorWheelRadius;
  double velAngularRight = velLinearRight / m_motorWheelRadius;

  ROS_INFO_STREAM("Twist x: " << _msg->linear.x << " z: " << _msg->angular.z
                              << " converted into left: "
                              << velAngularLeft
                              << " right: "
                              << velAngularRight);

  snd_proto::SerialRequest req;
  snd_proto::Speed* speed = req.mutable_setmotorsspeed();
  speed->set_left(static_cast<float>(velAngularLeft));
  speed->set_right(static_cast<float>(velAngularRight));
  m_com->sendMsg(req);
}

void BoardComm::readStatus()
{
  snd_proto::SerialRequest req;
  snd_proto::EmptyMsg* empty = req.mutable_getstatus();
  m_com->sendMsg(req);
  try
  {
    snd_proto::SerialResponse resp = m_com->readIncomingMsg();
    switch(resp.type_case())
    {
    case snd_proto::SerialResponse::kLog:
      switch(resp.log().level())
      {
      case snd_proto::Log::DEBUG: ROS_DEBUG_STREAM(resp.log().text()); break;
      case snd_proto::Log::INFO: ROS_INFO_STREAM(resp.log().text()); break;
      case snd_proto::Log::WARN: ROS_WARN_STREAM(resp.log().text()); break;
      case snd_proto::Log::ERROR: ROS_ERROR_STREAM(resp.log().text()); break;
      case snd_proto::Log::FATAL: ROS_FATAL_STREAM(resp.log().text()); break;
      }
      break;
    case snd_proto::SerialResponse::kEncoders:
      ROS_INFO_STREAM("Encoders left: " << resp.encoders().left() << " right: "
                                        << resp.encoders().right());
      {
        snd_msgs::Encoders msg;
        msg.header.stamp = ros::Time::now();
        msg.left = resp.encoders().left();
        msg.right = resp.encoders().right();
        m_encodersPub.publish(msg);
      }

      break;
    case snd_proto::SerialResponse::kPose:
      ROS_INFO_STREAM("Robot in position x:" << resp.pose().x() << " y: "
                                             << resp.pose().y()
                                             << " theta: "
                                             << resp.pose().th());
      break;
    case snd_proto::SerialResponse::kSpeed:
      ROS_INFO_STREAM("Motors speed left:" << resp.speed().left() << " right: "
                                           << resp.speed().right());
      break;
    case snd_proto::SerialResponse::kIsStarterSet:
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
    case snd_proto::SerialResponse::kStatus:
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
        snd_msgs::Encoders msg;
        msg.header.stamp = ros::Time::now();
        msg.left = resp.status().encoders().left();
        msg.right = resp.status().encoders().right();
        m_encodersPub.publish(msg);
      }
      break;

    default:
      ROS_WARN_STREAM("Message type: " << resp.type_case() << " not handled.");
    }
  }
  catch(const std::runtime_error& _e)
  {
    ROS_WARN_STREAM(_e.what());
  }
}

void BoardComm::dynParamCb(snd_serial::pidConfig& _config, uint32_t _level)
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
    snd_proto::SerialRequest req;
    snd_proto::PidTunings* pid = req.mutable_setpidspeedleft();
    pid->set_p(_config.left_speed_p);
    pid->set_i(_config.left_speed_i);
    pid->set_d(_config.left_speed_d);
    m_com->sendMsg(req);
  }
  // Right PID values modified
  if(_level & 0x2)
  {
    ROS_INFO_STREAM("Changing Reft Speed Pid to P: " << _config.right_speed_p
                                                     << " I: "
                                                     << _config.right_speed_i
                                                     << " D: "
                                                     << _config.right_speed_d);
    snd_proto::SerialRequest req;
    snd_proto::PidTunings* pid = req.mutable_setpidspeedright();
    pid->set_p(_config.right_speed_p);
    pid->set_i(_config.right_speed_i);
    pid->set_d(_config.right_speed_d);
    m_com->sendMsg(req);
  }
}

// vim: sw=2 ts=2 et
