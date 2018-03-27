#pragma once

#include "Motor.h"

#include "hal.h"

class L298 : public Motor
{
  friend class L298Motors;

public:
  L298(PWMDriver* _driver, const uint32_t _timerFrequency,
       const uint32_t _timerPeriod, const uint8_t _channelA,
       const bool _isAComplementaryChannel, const uint8_t _channelB,
       const bool _isBComplementaryChannel, stm32_gpio_t* _inAGpio,
       const uint32_t _inAPin, stm32_gpio_t* _inBGpio, const uint32_t _inBPin);
  void begin() override;
  void stop() override;
  void pwmI(int16_t _percentage) override;
  void brake() override;
  void setOutputPinsMode() override;
  void changeDirection(eDirection _direction) override;

protected:
  uint8_t m_channelA;
  const bool m_isAComplementaryChannel;
  uint8_t m_channelB;
  const bool m_isBComplementaryChannel;
  stm32_gpio_t* m_inAGpio;
  const uint32_t m_inAPin;
  stm32_gpio_t* m_inBGpio;
  const uint32_t m_inBPin;
};
