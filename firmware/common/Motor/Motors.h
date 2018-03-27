#pragma once

#include "Motor.h"

class Motors
{
public:
  Motors(Motor& _leftMotor, Motor& _rightMotor);
  virtual ~Motors() =  default;
  virtual void begin();
  void pwm(int16_t _leftPwm, int16_t _rightPwm);
  void pwmI(int16_t _leftPwm, int16_t _rightPwm);
  void stop();
  void brake();

protected:
  Motor& m_leftMotor;
  Motor& m_rightMotor;
  PWMConfig m_leftPwmConfig;
  PWMConfig m_rightPwmConfig;
  bool m_useSameDriver;
};
