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
  friend class Motors;
  friend class L298Motors;

public:
  enum eDirection
  {
    FORWARD,
    BACKWARD
  };


  Motor(PWMDriver* _driver, const uint32_t _timerFrequency,
        const uint32_t _timerPeriod, const uint8_t _channel,
        bool _isComplementaryChannel = false);
  virtual ~Motor() = default;
  virtual void begin();
  virtual void stop();
  virtual void pwm(int16_t _percentage);
  virtual void pwmI(int16_t _percentage);
  virtual void brake() = 0;
  virtual void changeDirection(eDirection _direction) = 0;
  virtual void setOutputPinsMode() = 0;

protected:
  PWMDriver* m_driver;
  const uint32_t m_timerFrequency;
  const uint32_t m_timerPeriod;
  uint8_t m_channel;
  bool m_isComplementaryChannel;
  PWMConfig m_pwmCfg;
};
