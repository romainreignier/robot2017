/*
 * Supemeca Never Dies 2017
 * \file Motor.cpp
 * \date 17/03/2017
 * \author Romain Reignier
 */

#include "Motor.h"
#include "Board.h"

#include "chprintf.h"

Motor::Motor(PWMDriver* _driver, uint32_t _channel,
             bool _isComplementaryChannel)
    : m_driver(_driver), m_channel(_channel - 1),
      m_pwmCfg{1000000, // Timer at 1 MHz
               1000,    // Period at 1000 so 1 kHz PWM
               NULL,    // pwm callback
               {{(_channel == 1) ? (_isComplementaryChannel
                                        ? PWM_COMPLEMENTARY_OUTPUT_ACTIVE_HIGH
                                        : PWM_OUTPUT_ACTIVE_HIGH)
                                 : PWM_OUTPUT_DISABLED,
                 NULL},
                {(_channel == 2) ? (_isComplementaryChannel
                                        ? PWM_COMPLEMENTARY_OUTPUT_ACTIVE_HIGH
                                        : PWM_OUTPUT_ACTIVE_HIGH)
                                 : PWM_OUTPUT_DISABLED,
                 NULL},
                {(_channel == 3) ? (_isComplementaryChannel
                                        ? PWM_COMPLEMENTARY_OUTPUT_ACTIVE_HIGH
                                        : PWM_OUTPUT_ACTIVE_HIGH)
                                 : PWM_OUTPUT_DISABLED,
                 NULL},
                {(_channel == 4) ? (_isComplementaryChannel
                                        ? PWM_COMPLEMENTARY_OUTPUT_ACTIVE_HIGH
                                        : PWM_OUTPUT_ACTIVE_HIGH)
                                 : PWM_OUTPUT_DISABLED,
                 NULL}},
               0,
               0}
{
}

void Motor::begin()
{
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
  _percentage = (_percentage > maxPwm) ? maxPwm : _percentage;
  pwmEnableChannel(
      m_driver, m_channel, PWM_PERCENTAGE_TO_WIDTH(m_driver, _percentage));
}
