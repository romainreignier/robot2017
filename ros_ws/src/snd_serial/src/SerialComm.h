#pragma once

#include <memory>

#include <ros/ros.h>
#include <serial/serial.h>
#include <geometry_msgs/Twist.h>

#include "CommMsgs.pb.h"

class SerialComm
{
public:
    SerialComm();
    ~SerialComm();
    void run();
    int sendMsg(const snd_msgs::SerialRequest& _req);
    void readIncomingMsg();
    void readStatus();

private:
    void cmdVelCb(const geometry_msgs::TwistConstPtr& _msg);

    ros::NodeHandle m_nh;
    std::unique_ptr<serial::Serial> m_serial;
    ros::Publisher m_encodersPub;
    ros::Publisher m_starterPub;
    ros::Publisher m_odomPub;
    ros::Subscriber m_cmdVelSub;

    double m_motorWheelSeparation;
    double m_motorWheelRadius;
    double m_encoderWheelSeparation;
    double m_encoderWheelRadius;
    double m_updateRate;
};
