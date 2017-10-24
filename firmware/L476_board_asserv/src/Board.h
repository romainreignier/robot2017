#pragma once

#include "hal.h"

#include "chprintf.h"

#include "Input.h"
#include "MonsterShield.h"
#include "Motors.h"
#include "Output.h"
#include "PCA9685.hpp"
#include "Qei.h"

#define SERIAL_DRIVER SD2
#define DEBUG_DRIVER SD2

#define PID_TIMER GPTD7

extern BaseSequentialStream* dbg;

#define USE_SERIAL_LOG

#if defined(USE_SERIAL_LOG)
#define DEBUG(...)                                                             \
  chprintf(dbg, __VA_ARGS__);                                                  \
  streamPut(dbg, '\n')
#else
#define DEBUG(...)
#endif

struct Board
{
  // Methods
  Board();
  void begin();
  void main();
  void startPIDTimer();
  void stopPIDTimer();
  void PIDTimerCb();
  void moveLinear(float _distance);
  void moveAngular(float _angle);
  void moveLinearEchelon(float _distance);
  void moveAngularEchelon(float _angle);
  void computeTraj();
  void asserv();
  void printErrors();
  int16_t boundPwm(int16_t _pwm);

  // helpers
  template <typename T> T bound(T _in, T _min, T _max);
  template <typename T> T boundServo(T _in, T _min, T _max);

  // Components
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
  Output tcsLed;

  uint16_t pidTimerPeriodMs = 20;

  volatile int32_t lastLeftTicks;
  volatile int32_t lastRightTicks;
  float cibleDistance;
  float cibleAngle;
  int16_t maxPwm;
  volatile float consigneDistance;
  volatile float consigneAngle;
  volatile float mesureDistance;
  volatile float mesureAngle;
  volatile float erreurDistance;
  volatile float erreurAngle;
  volatile float lastErreurDistance;
  volatile float lastErreurAngle;
  float kpDist;
  float kpAng;
  float kdDist;
  float kdAng;
  float kiDist;
  float kiAng;
  volatile float iTermDist;
  volatile float iTermAng;
  float iMinDist;
  float iMaxDist;
  float iMinAng;
  float iMaxAng;
  volatile uint32_t finAsservIterations;
  volatile bool finish;
  bool mustComputeTraj;
  volatile int16_t leftPwm;
  volatile int16_t rightPwm;

  float vLinMax;
  float vAngMax;

  static constexpr float kPi = 3.14159265358979323846;
  static constexpr float wheelSeparationMM = 102.1;
  static constexpr int32_t encoderResolution = 2400;
  static constexpr float leftWheelRadius = 26.125;
  static constexpr float rightWheelRadius = 26.075;
  static constexpr float LEFT_TICKS_TO_MM =
    (2 * kPi * leftWheelRadius) / encoderResolution;
  static constexpr float RIGHT_TICKS_TO_MM =
    (2 * kPi * rightWheelRadius) / encoderResolution;
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