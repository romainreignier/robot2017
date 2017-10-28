/*
 * Supemeca Never Dies 2017
 * \file ArdumotoShield.h
 * \date 18/03/2017
 * \author Romain Reignier
 */

#pragma once

#include "Motor.h"
#include "hal.h"

class ArdumotoShield : public Motor
{
public:
  ArdumotoShield(PWMDriver* _driver, const uint32_t _timerFrequency,
                 const uint32_t _timerPeriod, const uint8_t _channel,
                 const bool _isComplementaryChannel, stm32_gpio_t* _dirGpio,
                 const uint32_t _dirPin);
  virtual void brake() override;
  virtual void changeDirection(eDirection _direction) override;
  virtual void setOutputPinsMode() override;

protected:
  stm32_gpio_t* m_dirGpio;
  const uint32_t m_dirPin;
};
