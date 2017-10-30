#pragma once

#include "hal.h"

#include "chprintf.h"

#include "AdcTimer.h"
#include "Input.h"
#include "Output.h"
#include "Pid.h"
#include "Qei.h"
#include "RunningAverage.h"

#define USE_L298
#if defined(USE_MONSTER_SHIELD)
#include "MonsterShield.h"
#include "Motors.h"
#elif defined(USE_L298)
#include "L298.h"
#include "L298Motors.h"
#endif

#include <ros.h>
#include <snd_msgs/Encoders.h>
#include <snd_msgs/Motors.h>
#include <snd_msgs/Pid.h>
#include <snd_msgs/Status.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int16.h>
#include <std_msgs/UInt16.h>

#define SERIAL_DRIVER SD2
#define DEBUG_DRIVER SD1

#define PID_TIMER GPTD7
#define ENCODER_COUNT_TIMER GPTD6

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
struct Board
{
  // Methods
  Board();
  void begin();
  // ROS publish methods
  void publishFeedback();
  void publishStatus();
  // ROS Callbacks
  void motorsSpeedCb(const snd_msgs::Motors& _msg);
  void motorsModeCb(const snd_msgs::MotorControlMode& _msg);
  void leftMotorPwmCb(const std_msgs::Int16& _msg);
  void rightMotorPwmCb(const std_msgs::Int16& _msg);
  void leftMotorPidCb(const snd_msgs::Pid& _msg);
  void rightMotorPidCb(const snd_msgs::Pid& _msg);
  void resetStatusCb(const std_msgs::Empty& _msg);

  void checkMotorsCurrent();
  void motorsControl();

  // helpers
  template <typename T> T bound(T _in, T _min, T _max);

  // Components
  // Clock @ 48 MHz, Timer @ 16 MHz, Period = 1,2 k -> PWM @ 13 kHz
  static constexpr uint32_t kPwmTimerFrequency{16'000'000};
  static constexpr uint16_t kPwmTimerPeriod{1200};
#if defined(USE_MONSTER_SHIELD)
  MonsterShield leftMotor;
  MonsterShield rightMotor;
  Motors motors;
#elif defined(USE_L298)
  L298 leftMotor;
  L298 rightMotor;
  L298Motors motors;
#endif
  Qei qei;
  Input starter;
  Input colorSwitch;
  Input eStop;

  // AdcTimer motorsCurrentChecker;
  systime_t timeStartOverCurrent = 0;
  uint8_t globalStatus;
  static constexpr uint16_t kCurrentThreshold = 6000;
  static constexpr systime_t kMaxTimeOverCurrent = MS2ST(1000);
  // From VNH2SP30 datasheet
  // (Vmax (mV) * ratio Iout/Isense) / (maxAdc * RSense)
  static constexpr float kAdcToMilliAmps = (3300 * 11370) / (4095 * 1500);

  uint16_t pidTimerPeriodMs = 10;
  PID leftMotorPid;
  PID rightMotorPid;
  float leftMotorSpeed;
  float leftMotorPwm;
  float leftMotorCommand;
  float rightMotorSpeed;
  float rightMotorPwm;
  float rightMotorCommand;
  snd_msgs::MotorControlMode motorsMode;
  bool mustPublishFeedback = false;
  // 500 kHz -> 1 timer tick = 2 µs
  static constexpr uint32_t kQeiTimerFrequency{500000};
  RunningAverage<int32_t, 2> leftQeiAvg;
  RunningAverage<int32_t, 2> rightQeiAvg;
  int32_t leftQeiCnt = 0;
  int32_t rightQeiCnt = 0;

  // ROS
  ros::NodeHandle nh;
  // Publishers and messages
  snd_msgs::Status statusMsg;
  ros::Publisher statusPub;
  snd_msgs::Encoders encodersMsg;
  ros::Publisher encodersPub;
  snd_msgs::Motors motorsCurrentMsg;
  ros::Publisher motorsCurrentPub;
  // Subscribers
  ros::Subscriber<snd_msgs::Motors, Board> motorsSpeedSub;
  ros::Subscriber<snd_msgs::MotorControlMode, Board> motorsModeSub;
  ros::Subscriber<std_msgs::Int16, Board> leftMotorPwmSub;
  ros::Subscriber<std_msgs::Int16, Board> rightMotorPwmSub;
  ros::Subscriber<snd_msgs::Pid, Board> leftMotorPidSub;
  ros::Subscriber<snd_msgs::Pid, Board> rightMotorPidSub;
  ros::Subscriber<std_msgs::Empty, Board> resetStatusSub;
};

template <typename T> T Board::bound(T _in, T _min, T _max)
{
  if(_in > _max) return _max;
  if(_in < _min) return _min;
  return _in;
}

// Global instance of the board struct
extern Board gBoard;
