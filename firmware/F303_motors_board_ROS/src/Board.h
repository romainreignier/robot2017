#pragma once

#include <ros.h>
#include <std_msgs/Bool.h>
#include "MonsterShield.h"
#include "Qei.h"
#include "Input.h"
#include "Motors.h"

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
  void publishAll();

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
  std_msgs::Bool starterMsg;
  ros::Publisher starterPub;
  /*
  ros::Publisher eStopPub;
  ros::Publisher colorSwitchPub;
  ros::Publisher encodersPub;
  ros::Publisher motorsCurrentPub;
  ros::Publisher motorsSpeedPub;
  */
};

extern Board gBoard;
