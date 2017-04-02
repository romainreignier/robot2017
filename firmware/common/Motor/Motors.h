#pragma once

class Motor;
#include "Motor.h"

class Motors
{
public:
  Motors(Motor& _leftMotor, Motor& _rightMotor);
  void begin();
  void pwm(int16_t _leftPwm, int16_t _rightPwm);
  void stop();

protected:
  Motor& m_leftMotor;
  Motor& m_rightMotor;
  PWMConfig m_leftPwmConfig;
  PWMConfig m_rightPwmConfig;
  bool m_useSameDriver;
};
