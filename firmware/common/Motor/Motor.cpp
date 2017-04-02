/*
 * Supemeca Never Dies 2017
 * \file Motor.cpp
 * \date 17/03/2017
 * \author Romain Reignier
 */

#include "Motor.h"
#include "Board.h"

#include "chprintf.h"

Motor::Motor(PWMDriver* _driver, const uint8_t _channel,
             bool _isComplementaryChannel)
  : m_driver{_driver}, m_channel(_channel - 1),
    m_isComplementaryChannel(_isComplementaryChannel)
{
}

void Motor::begin()
{
  m_pwmCfg.frequency = kPwmFrequency;
  m_pwmCfg.period = kPwmPeriod;
  m_pwmCfg.callback = NULL; // no pwm callback
  m_pwmCfg.channels[m_channel].mode =
    (m_isComplementaryChannel ? PWM_COMPLEMENTARY_OUTPUT_ACTIVE_HIGH
                              : PWM_OUTPUT_ACTIVE_HIGH);

  setOutputPinsMode();
  pwmStart(m_driver, &m_pwmCfg);
  stop();
}

void Motor::stop()
{
  pwmDisableChannel(m_driver, m_channel);
}

void Motor::pwm(int16_t _percentage)
{
  chprintf(dbg, "set pwm %d\n", _percentage);
  if(_percentage >= 0)
  {
    changeDirection(FORWARD);
  }
  else
  {
    changeDirection(BACKWARD);
    _percentage *= -1;
  }
  _percentage = (_percentage > kPwmPeriod) ? kPwmPeriod : _percentage;
  pwmEnableChannel(
    m_driver, m_channel, PWM_PERCENTAGE_TO_WIDTH(m_driver, _percentage));
}
