/*
 * Supemeca Never Dies 2017
 * \file Motor.cpp
 * \date 17/03/2017
 * \author Romain Reignier
 */

#include "Motor.h"
#include "Board.h"

Motor::Motor(PWMDriver* _driver, const uint32_t _timerFrequency,
             const uint32_t _timerPeriod, const uint8_t _channel,
             bool _isComplementaryChannel)
  : m_driver{_driver}, m_timerFrequency{_timerFrequency},
    m_timerPeriod{_timerPeriod}, m_channel(_channel - 1),
    m_isComplementaryChannel(_isComplementaryChannel)
{
}

void Motor::begin()
{
  m_pwmCfg.frequency = m_timerFrequency;
  m_pwmCfg.period = m_timerPeriod;
  m_pwmCfg.callback = NULL;
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
  osalSysLock();
  pwmI(_percentage);
  osalSysUnlock();
}

void Motor::pwmI(int16_t _percentage)
{
  if(_percentage > 0)
  {
    changeDirection(FORWARD);
  }
  else if(_percentage < 0)
  {
    changeDirection(BACKWARD);
    _percentage *= -1;
  }
  _percentage = (_percentage > 10000) ? 10000 : _percentage;
  pwmEnableChannelI(
    m_driver, m_channel, PWM_PERCENTAGE_TO_WIDTH(m_driver, static_cast<uint16_t>(_percentage)));
}
