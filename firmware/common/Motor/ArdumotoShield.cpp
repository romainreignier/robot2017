/*
 * Supemeca Never Dies 2017
 * \file ArdumotoShield.cpp
 * \date 18/03/2017
 * \author Romain Reignier
 */

#include "ArdumotoShield.h"
#include "Board.h"

#include "chprintf.h"

ArdumotoShield::ArdumotoShield(PWMDriver* _driver, uint8_t _channel,
                               bool _isComplementaryChannel,
                               stm32_gpio_t* _dirGpio, const uint32_t _dirPin)
    : Motor{_driver, _channel, _isComplementaryChannel}, m_dirGpio{_dirGpio},
      m_dirPin{_dirPin}
{
}

void ArdumotoShield::changeDirection(eDirection _direction)
{
  if(_direction == FORWARD)
  {
    palSetPad(m_dirGpio, m_dirPin);
    chprintf(dbg, "set pin forward\n");
  }
  else
  {
    palClearPad(m_dirGpio, m_dirPin);
    chprintf(dbg, "set pin backward\n");
  }
}

void ArdumotoShield::setOutputPinsMode()
{
  palSetPadMode(m_dirGpio, m_dirPin, PAL_MODE_OUTPUT_PUSHPULL);
}
