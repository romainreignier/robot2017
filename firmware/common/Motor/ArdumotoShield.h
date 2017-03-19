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
  ArdumotoShield(PWMDriver* _driver, uint32_t _channel,
                 bool _isComplementaryChannel, stm32_gpio_t* _dirGpio,
                 uint32_t _dirPin);
  virtual void changeDirection(eDirection _direction) override;
  virtual void setOutputPinsMode() override;

  protected:
  stm32_gpio_t* m_dirGpio;
  uint32_t m_dirPin;
};
