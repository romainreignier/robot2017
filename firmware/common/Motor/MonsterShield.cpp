/*
 * Supemeca Never Dies 2017
 * \file MonsterShield.cpp
 * \date 18/03/2017
 * \author Romain Reignier
 *
 * Driver for the ST VNH2SP30 DC Motor Driver
 * Truth table:
 * CWPIN  CCWPIN  Fonction
 * ---------------------------------
 *   0      0     Brake GND
 *   1      0     Clock Wise
 *   1      1     Brake VCC
 *   0      1     Counter Clock Wise
 */

#include "MonsterShield.h"

MonsterShield::MonsterShield(PWMDriver* _driver, uint32_t _channel,
                             bool _isComplementaryChannel,
                             stm32_gpio_t* _CWGpio, uint32_t _CWPin,
                             stm32_gpio_t* _CCWGpio, uint32_t _CCWPin)
    : Motor(_driver, _channel, _isComplementaryChannel), m_CWGpio(_CWGpio),
      m_CWPin(_CWPin), m_CCWGpio(_CCWGpio), m_CCWPin(_CCWPin)
{
}

void MonsterShield::changeDirection(eDirection _direction)
{
  if(_direction == FORWARD)
  {
    palClearPad(m_CWGpio, m_CWPin);
    palSetPad(m_CCWGpio, m_CCWPin);
  }
  else
  {
    palClearPad(m_CCWGpio, m_CCWPin);
    palSetPad(m_CWGpio, m_CWPin);
  }
}

void MonsterShield::setOutputPinsMode()
{
  palSetPadMode(m_CWGpio, m_CWPin, PAL_MODE_OUTPUT_PUSHPULL);
  palSetPadMode(m_CCWGpio, m_CCWPin, PAL_MODE_OUTPUT_PUSHPULL);
}

void MonsterShield::brake()
{
  stop();
  palSetPad(m_CWGpio, m_CWPin);
  palSetPad(m_CCWGpio, m_CCWPin);
}
