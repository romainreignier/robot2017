#include "L298Motors.h"

L298Motors::L298Motors(L298& _leftMotor, L298& _rightMotor)
  : Motors{_leftMotor, _rightMotor}
{
}

void L298Motors::begin()
{
  // Only implement the 2 motors using the same PWM driver (timer)
  L298 &leftMotor = static_cast<L298&>(m_leftMotor);
  L298 &rightMotor = static_cast<L298&>(m_rightMotor);

  // Only use the left config for both motors
  m_leftPwmConfig.frequency = m_leftMotor.m_timerFrequency;
  m_leftPwmConfig.period = m_leftMotor.m_timerPeriod;
  m_leftPwmConfig.callback = NULL; // no pwm callback
  m_leftPwmConfig.channels[leftMotor.m_channelA].mode =
    (leftMotor.m_isAComplementaryChannel
       ? PWM_COMPLEMENTARY_OUTPUT_ACTIVE_HIGH
       : PWM_OUTPUT_ACTIVE_HIGH);
  m_leftPwmConfig.channels[leftMotor.m_channelB].mode =
    (leftMotor.m_isBComplementaryChannel
       ? PWM_COMPLEMENTARY_OUTPUT_ACTIVE_HIGH
       : PWM_OUTPUT_ACTIVE_HIGH);
  m_leftPwmConfig.channels[rightMotor.m_channelA].mode =
    (rightMotor.m_isAComplementaryChannel
       ? PWM_COMPLEMENTARY_OUTPUT_ACTIVE_HIGH
       : PWM_OUTPUT_ACTIVE_HIGH);
  m_leftPwmConfig.channels[rightMotor.m_channelB].mode =
    (rightMotor.m_isBComplementaryChannel
       ? PWM_COMPLEMENTARY_OUTPUT_ACTIVE_HIGH
       : PWM_OUTPUT_ACTIVE_HIGH);

  pwmStart(m_leftMotor.m_driver, &m_leftPwmConfig);

  stop();
}
