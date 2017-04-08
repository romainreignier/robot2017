#pragma once

#include <memory>

#include <ros/ros.h>
#include <serial/serial.h>
#include <geometry_msgs/Twist.h>
#include <dynamic_reconfigure/server.h>
#include <snd_serial/pidConfig.h>

#include "CommMsgs.pb.h"

class SerialComm
{
public:
    SerialComm();
    ~SerialComm();
    void run();
    int sendMsg(const snd_proto::SerialRequest& _req);
    void readIncomingMsg();
    void readStatus();

private:
    void cmdVelCb(const geometry_msgs::TwistConstPtr& _msg);
    void dynParamCb(snd_serial::pidConfig& _config, uint32_t _level);

    ros::NodeHandle m_nh;
    std::unique_ptr<serial::Serial> m_serial;
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
