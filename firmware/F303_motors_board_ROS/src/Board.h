#pragma once

#include "hal.h"

#include "chprintf.h"

#include "AdcTimer.h"
#include "Input.h"
#include "MonsterShield.h"
#include "Motors.h"
#include "Pid.h"
#include "Qei.h"

#include <ros.h>
#include <snd_msgs/Encoders.h>
#include <snd_msgs/Motors.h>
#include <snd_msgs/Pid.h>
#include <snd_msgs/Status.h>
#include <std_msgs/Empty.h>

#define SERIAL_DRIVER SD2
#define DEBUG_DRIVER SD1

extern BaseSequentialStream* dbg;

// #define USE_SERIAL_LOG
#define USE_ROS_LOG

#if defined(USE_SERIAL_LOG)
#define DEBUG(...)                                                             \
  chprintf(dbg, __VA_ARGS__);                                                  \
  streamPut(dbg, '\n')
#elif defined(USE_ROS_LOG)
#define LOG_BUFFER_SIZE 128
extern char logBuffer[LOG_BUFFER_SIZE];
#define DEBUG(...)                                                             \
  chsnprintf(logBuffer, LOG_BUFFER_SIZE, __VA_ARGS__);                         \
  gBoard.nh.loginfo(logBuffer)
#else
#define DEBUG(...)
#endif

// Here we use a struct instead of a class to ease the use of the object
// So no need to use getters
// Note that a Singleton Design Pattern could be used but
// it needs dynamic memory allocation to be used.
struct Board
{
  // Methods
  Board();
  void begin();
  void publishFeedback();
  void publishStatus();
  void motorsSpeedCb(const snd_msgs::Motors& _msg);
  void leftMotorPidCb(const snd_msgs::Pid& _msg);
  void rightMotorPidCb(const snd_msgs::Pid& _msg);
  void resetStatusCb(const std_msgs::Empty& _msg);
  void checkMotorsCurrent();
  void motorsControl();

  // Components
  MonsterShield leftMotor;
  MonsterShield rightMotor;
  Motors motors;
  Qei qei;
  Input starter;
  Input colorSwitch;
  Input eStop;
  AdcTimer motorsCurrentChecker;

  uint16_t pidTimerPeriodMs;
  PID leftMotorPid;
  PID rightMotorPid;
  float leftMotorSpeed;
  float leftMotorPwm;
  float leftMotorCommand;
  float rightMotorSpeed;
  float rightMotorPwm;
  float rightMotorCommand;
  int32_t lastLeftTicks;
  int32_t lastRightTicks;

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
  ros::Subscriber<std_msgs::Empty, Board> resetStatusSub;

  uint8_t globalStatus;
  systime_t timeStartOverCurrent;
  static constexpr uint16_t kCurrentThreshold = 6000;
  static constexpr systime_t kMaxTimeOverCurrent = MS2ST(1000);
  // (Vmax (mV) * ratio Iout/Isense) / (maxAdc * RSense)
  static constexpr float kAdcToMilliAmps = (3300 * 11370) / (4095 * 1500);
};

extern Board gBoard;
