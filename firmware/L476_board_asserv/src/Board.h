#pragma once

#include "hal.h"

#include "chprintf.h"

#include "Input.h"
#include "MonsterShield.h"
#include "Motors.h"
#include "Output.h"
#include "PCA9685.hpp"
#include "Qei.h"
#include "RunningAverage.h"

#define DTOR 0.0174532925199433
#define RTOD 57.29577951308230

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
  void lectureCodeur();
  float normalize_angle(float angle);
  float normalize_angle_positive(float angle);
  void needMotorGraph();
  void SetInitPosition(const float & pX, const float & pY, const float & pTheta);

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

  static constexpr uint32_t kPidTimerPeriodMs = 10;
  static constexpr uint32_t kQeiTimerFrequency = 500000;

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
  volatile int16_t cptRestOnPosition;
  volatile int16_t cptRestOnAngle;
  volatile int16_t compensationDist;
  volatile int16_t compensationAng;
  float kpDist;
  float kpAng;
  float kiDist;
  float kiAng;
  float kdDist;
  float kdAng;
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
  float leftSpeed;
  float rightSpeed;
  RunningAverage<int32_t, 2> leftQeiAvg;
  RunningAverage<int32_t, 2> rightQeiAvg;
  int32_t leftQeiCnt = 0;
  int32_t rightQeiCnt = 0;

  float vLinMax;
  float vAngMax;

  float smoothRotation;
  float linear_speed;

  static constexpr float kPi = 3.14159265358979323846f;
  static constexpr float wheelSeparationMM = 102.1f;
  static constexpr int32_t encoderResolution = 2400;
  static constexpr float leftWheelRadius = 26.125f;
  static constexpr float rightWheelRadius = 26.075f;
  static constexpr float LEFT_TICKS_TO_MM =
    (2 * kPi * leftWheelRadius) / encoderResolution;
  static constexpr float RIGHT_TICKS_TO_MM =
    (2 * kPi * rightWheelRadius) / encoderResolution;

  float G_X_mm;
  float G_Y_mm;
  float G_Theta_rad;
  float dD;
  float dA;
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
