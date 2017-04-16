#pragma once

#include "Input.h"
#include "MonsterShield.h"
#include "Motors.h"
#include "Qei.h"
#include <ros.h>
#include <snd_msgs/Encoders.h>
#include <snd_msgs/Motors.h>
#include <snd_msgs/Pid.h>
#include <snd_msgs/Status.h>

#define SERIAL_DRIVER SD2
#define DEBUG_DRIVER SD1

extern BaseSequentialStream* dbg;
extern BaseSequentialStream* ser;

// Here we use a struct instead of a class to ease the use of the object
// So no need to use getters
// Note that a Singleton Design Pattern could be used but
// it needs dynamic memory allocation to be used.
struct Board
{
  Board();
  void begin();
  void publishFeedback();
  void publishStatus();
  void motorsSpeedCb(const snd_msgs::Motors& _msg);
  void leftMotorPidCb(const snd_msgs::Pid& _msg);
  void rightMotorPidCb(const snd_msgs::Pid& _msg);

  // Components
  MonsterShield leftMotor;
  MonsterShield rightMotor;
  Motors motors;
  Qei qei;
  Input starter;
  Input colorSwitch;
  Input eStop;

  // ROS
  ros::NodeHandle nh;
  // Publishers and messages
  snd_msgs::Status statusMsg;
  ros::Publisher statusPub;
  snd_msgs::Encoders encodersMsg;
  ros::Publisher encodersPub;
  // Subscribers
  ros::Subscriber<snd_msgs::Motors, Board> motorsSpeedSub;
  ros::Subscriber<snd_msgs::Pid, Board> leftMotorPidSub;
  ros::Subscriber<snd_msgs::Pid, Board> rightMotorPidSub;
};

extern Board gBoard;
