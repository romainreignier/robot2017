#pragma once

#include "hal.h"

#include "chprintf.h"

#include "AdcTimer.h"
#include "Input.h"
#include "MonsterShield.h"
#include "Motors.h"
#include "Output.h"
#include "PCA9685.hpp"
#include "Pid.h"
#include "Qei.h"
#include "Adafruit_TCS34725.h"
#include "VL53L0X.h"

#include <ros.h>
#include <snd_msgs/Encoders.h>
#include <snd_msgs/MotorControlMode.h>
#include <snd_msgs/Motors.h>
#include <snd_msgs/Pid.h>
#include <snd_msgs/ProximitySensors.h>
#include <snd_msgs/Status.h>
#include <std_msgs/Bool.h>
#include <std_msgs/ColorRGBA.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int16.h>
#include <std_msgs/UInt16.h>

#define SERIAL_DRIVER SD2
#define DEBUG_DRIVER SD1

#define PID_TIMER GPTD7

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
  void main();
  void checkMotorsCurrent();
  void motorsControl();
  void startPIDTimer();
  void stopPIDTimer();

  // ROS publish methods
  void publishFeedback();
  void publishStatus();
  void publishSensors();
  // ROS Callbacks
  void motorsSpeedCb(const snd_msgs::Motors& _msg);
  void motorsModeCb(const snd_msgs::MotorControlMode& _msg);
  void leftMotorPwmCb(const std_msgs::Int16& _msg);
  void rightMotorPwmCb(const std_msgs::Int16& _msg);
  void leftMotorPidCb(const snd_msgs::Pid& _msg);
  void rightMotorPidCb(const snd_msgs::Pid& _msg);
  void resetStatusCb(const std_msgs::Empty& _msg);
  void armServoCb(const std_msgs::UInt16& _msg);
  void grasp1ServoCb(const std_msgs::UInt16& _msg);
  void grasp2ServoCb(const std_msgs::UInt16& _msg);
  void pumpCb(const std_msgs::Bool& _msg);
  void greenLedCb(const std_msgs::Bool& _msg);
  void launchServoCb(const std_msgs::UInt16& _msg);
  void ramp1ServoCb(const std_msgs::UInt16& _msg);
  void ramp2ServoCb(const std_msgs::UInt16& _msg);

  // helpers
  template <typename T> T bound(T _in, T _min, T _max);
  template <typename T> T boundServo(T _in, T _min, T _max);

  // Components
  static constexpr uint32_t kPwmTimerFrequency{10'000'000};
  static constexpr uint16_t kPwmTimerPeriod{1000};
  MonsterShield leftMotor;
  MonsterShield rightMotor;
  Motors motors;
  Qei qei;
  Input starter;
  Input colorSwitch;
  Input selector;
  Input eStop;
  Input frontProximitySensor;
  Input rearLeftProximitySensor;
  Input rearRightProximitySensor;
  Output pump;
  Output greenLed;
  PCA9685 servos;
  const uint8_t kArmServoId = 15;
  const uint8_t kGrasp1ServoId = 7;
  const uint8_t kGrasp2ServoId = 6;
  const uint8_t kLaunchServoId = 0;
  const uint8_t kRamp1ServoId = 8;
  const uint8_t kRamp2ServoId = 9;
  // this is the 'minimum' pulse length count
  static constexpr uint16_t kServoMin = 100;
  // this is the 'maximum' pulse length count
  static constexpr uint16_t kServoMax = 700;
  // VL53L0X leftVlx;
  Adafruit_TCS34725 tcs;
  Output tcsLed;

  AdcTimer motorsCurrentChecker;
  systime_t timeStartOverCurrent;
  systime_t timeLastStatus;
  systime_t timeLastSensors;
  uint8_t globalStatus;
  static constexpr uint16_t kCurrentThreshold = 6000;
  static constexpr systime_t kMaxTimeOverCurrent = TIME_MS2I(1000);
  // (Vmax (mV) * ratio Iout/Isense) / (maxAdc * RSense)
  static constexpr float kAdcToMilliAmps = (3300 * 11370) / (4095 * 1500);
  static constexpr systime_t kStatusPeriod = TIME_MS2I(500);
  static constexpr systime_t kSensorsPeriod = TIME_MS2I(50);
  static constexpr systime_t kFeedbackPeriod = TIME_MS2I(25);

  uint16_t pidTimerPeriodMs = 10;
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
  snd_msgs::MotorControlMode motorsMode;
  float leftMotorCorrector;
  float rightMotorCorrector;

  // ROS
  ros::NodeHandle nh;
  // Publishers and messages
  snd_msgs::Status statusMsg;
  ros::Publisher statusPub;
  snd_msgs::Encoders encodersMsg;
  ros::Publisher encodersPub;
  snd_msgs::Encoders commandsMsg;
  ros::Publisher commandsPub;
  std_msgs::ColorRGBA colorSensorMsg;
  ros::Publisher colorSensorPub;
  snd_msgs::ProximitySensors proximitySensorsMsg;
  ros::Publisher proximitySensorsPub;
  // Subscribers
  ros::Subscriber<snd_msgs::Motors, Board> motorsSpeedSub;
  ros::Subscriber<snd_msgs::MotorControlMode, Board> motorsModeSub;
  ros::Subscriber<std_msgs::Int16, Board> leftMotorPwmSub;
  ros::Subscriber<std_msgs::Int16, Board> rightMotorPwmSub;
  ros::Subscriber<snd_msgs::Pid, Board> leftMotorPidSub;
  ros::Subscriber<snd_msgs::Pid, Board> rightMotorPidSub;
  ros::Subscriber<std_msgs::Empty, Board> resetStatusSub;
  ros::Subscriber<std_msgs::UInt16, Board> armServoSub;
  ros::Subscriber<std_msgs::UInt16, Board> grasp1ServoSub;
  ros::Subscriber<std_msgs::UInt16, Board> grasp2ServoSub;
  ros::Subscriber<std_msgs::Bool, Board> pumpSub;
  ros::Subscriber<std_msgs::Bool, Board> greenLedSub;
  ros::Subscriber<std_msgs::UInt16, Board> launchServoSub;
  ros::Subscriber<std_msgs::UInt16, Board> ramp1ServoSub;
  ros::Subscriber<std_msgs::UInt16, Board> ramp2ServoSub;
};

template <typename T> T Board::bound(T _in, T _min, T _max)
{
  if(_in > _max) return _max;
  if(_in < _min) return _min;
  return _in;
}

// 4096 means OFF for pca
template <typename T> T Board::boundServo(T _in, T _min, T _max)
{
    if(_in == 0)
    {
        return 4096;
    }
    else
    {
        return bound(_in, _min, _max);
    }
}

// Global instance of the board struct
extern Board gBoard;
