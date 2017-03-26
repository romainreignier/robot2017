/*
 * Supemeca Never Dies 2017
 * \file Motor.h
 * \date 17/03/2017
 * \author Romain Reignier
 */

#pragma once

#include "hal.h"

class Motor
{
  public:
  enum eDirection
  {
    FORWARD,
    BACKWARD
  };
  static constexpr uint16_t maxPwm{10000};
  // TODO: change direction security, not too often
  Motor(PWMDriver* _driver, const uint8_t _channel, bool _isComplementaryChannel = false);
  void begin();
  void stop();
  void pwm(int16_t _percentage);
  virtual void changeDirection(eDirection _direction) = 0;
  virtual void setOutputPinsMode() = 0;

  protected:
  PWMDriver* m_driver;
  uint8_t m_channel;
  PWMConfig m_pwmCfg;
};
