/*
 * Supemeca Never Dies 2017
 * \file MonsterShield.h
 * \date 18/03/2017
 * \author Romain Reignier
 */

#pragma once

#include "Motor.h"
#include "hal.h"

class MonsterShield : public Motor
{
  public:
  MonsterShield(PWMDriver* _driver, uint8_t _channel,
                bool _isComplementaryChannel, stm32_gpio_t* _CWGpio,
                const uint32_t _CWPin, stm32_gpio_t* _CCWGpio, const uint32_t _CCWPin);
  virtual void changeDirection(eDirection _direction) override;
  virtual void setOutputPinsMode() override;
  void brake();

  protected:
  stm32_gpio_t* m_CWGpio;
  const uint32_t m_CWPin;
  stm32_gpio_t* m_CCWGpio;
  const uint32_t m_CCWPin;
};
