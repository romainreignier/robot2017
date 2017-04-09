/*
 * Supemeca Never Dies 2017
 * \file Motor.h
 * \date 17/03/2017
 * \author Romain Reignier
 */

#pragma once

#include "hal.h"

#include "Motors.h"

class Motor
{
  friend class Motors;
public:
  enum eDirection
  {
    FORWARD,
    BACKWARD
  };
  static constexpr uint32_t kPwmFrequency{1000000};
  static constexpr uint16_t kPwmPeriod{1000};
  // TODO: change direction security, not too often
  Motor(PWMDriver* _driver, const uint8_t _channel,
        bool _isComplementaryChannel = false);
  void begin();
  void stop();
  void pwm(int16_t _percentage);
  virtual void brake() = 0;
  virtual void changeDirection(eDirection _direction) = 0;
  virtual void setOutputPinsMode() = 0;

protected:
  PWMDriver* m_driver;
  uint8_t m_channel;
  bool m_isComplementaryChannel;
  PWMConfig m_pwmCfg;
};
