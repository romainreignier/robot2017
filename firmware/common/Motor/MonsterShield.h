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
  MonsterShield(PWMDriver* _driver, const uint32_t _timerFrequency,
                const uint32_t _timerPeriod, const uint8_t _channel,
                const bool _isComplementaryChannel, stm32_gpio_t* _CWGpio,
                const uint32_t _CWPin, stm32_gpio_t* _CCWGpio,
                const uint32_t _CCWPin, stm32_gpio_t* _ENGpio,
                const uint32_t _ENPin);
  virtual void brake() override;
  virtual void changeDirection(eDirection _direction) override;
  virtual void setOutputPinsMode() override;

protected:
  stm32_gpio_t* m_CWGpio;
  const uint32_t m_CWPin;
  stm32_gpio_t* m_CCWGpio;
  const uint32_t m_CCWPin;
  stm32_gpio_t* m_ENGpio;
  const uint32_t m_ENPin;
};
