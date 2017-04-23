#include "Motors.h"

Motors::Motors(Motor& _leftMotor, Motor& _rightMotor)
  : m_leftMotor(_leftMotor), m_rightMotor(_rightMotor)
{
}

void Motors::begin()
{
  // Checks if the 2 motors use the same PWM Driver, because we need to init
  // them at the same time if is the case
  if(m_leftMotor.m_driver == m_rightMotor.m_driver)
  {
    m_useSameDriver = true;
    // Only use the left config for both motors
    m_leftPwmConfig.frequency = Motor::kPwmFrequency;
    m_leftPwmConfig.period = Motor::kPwmPeriod;
    m_leftPwmConfig.callback = NULL; // no pwm callback
    m_leftPwmConfig.channels[m_leftMotor.m_channel].mode =
      (m_leftMotor.m_isComplementaryChannel
         ? PWM_COMPLEMENTARY_OUTPUT_ACTIVE_HIGH
         : PWM_OUTPUT_ACTIVE_HIGH);
    m_leftPwmConfig.channels[m_rightMotor.m_channel].mode =
      (m_rightMotor.m_isComplementaryChannel
         ? PWM_COMPLEMENTARY_OUTPUT_ACTIVE_HIGH
         : PWM_OUTPUT_ACTIVE_HIGH);
  }
  else
  {
    m_useSameDriver = false;
    m_leftPwmConfig.frequency = Motor::kPwmFrequency;
    m_leftPwmConfig.period = Motor::kPwmPeriod;
    m_leftPwmConfig.callback = NULL; // no pwm callback
    m_leftPwmConfig.channels[m_leftMotor.m_channel].mode =
      (m_leftMotor.m_isComplementaryChannel
         ? PWM_COMPLEMENTARY_OUTPUT_ACTIVE_HIGH
         : PWM_OUTPUT_ACTIVE_HIGH);

    m_rightPwmConfig.frequency = Motor::kPwmFrequency;
    m_rightPwmConfig.period = Motor::kPwmPeriod;
    m_rightPwmConfig.callback = NULL; // no pwm callback
    m_rightPwmConfig.channels[m_rightMotor.m_channel].mode =
      (m_rightMotor.m_isComplementaryChannel
         ? PWM_COMPLEMENTARY_OUTPUT_ACTIVE_HIGH
         : PWM_OUTPUT_ACTIVE_HIGH);
  }

  m_leftMotor.setOutputPinsMode();
  m_rightMotor.setOutputPinsMode();
  if(m_useSameDriver)
  {
    pwmStart(m_leftMotor.m_driver, &m_leftPwmConfig);
  }
  else
  {
    pwmStart(m_leftMotor.m_driver, &m_leftPwmConfig);
    pwmStart(m_rightMotor.m_driver, &m_leftPwmConfig);
  }
  stop();
}

void Motors::pwm(int16_t _leftPwm, int16_t _rightPwm)
{
  m_leftMotor.pwm(_leftPwm);
  m_rightMotor.pwm(_rightPwm);
}

void Motors::pwmI(int16_t _leftPwm, int16_t _rightPwm)
{
  m_leftMotor.pwmI(_leftPwm);
  m_rightMotor.pwmI(_rightPwm);
}

void Motors::stop()
{
  m_leftMotor.stop();
  m_rightMotor.stop();
}

void Motors::brake()
{
  m_leftMotor.brake();
  m_rightMotor.brake();
}
