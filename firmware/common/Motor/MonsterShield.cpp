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

MonsterShield::MonsterShield(PWMDriver* _driver, uint8_t _channel,
                             bool _isComplementaryChannel,
                             stm32_gpio_t* _CWGpio, const uint32_t _CWPin,
                             stm32_gpio_t* _CCWGpio, const uint32_t _CCWPin,
                             stm32_gpio_t* _ENGpio, const uint32_t _ENPin)
  : Motor{_driver, _channel, _isComplementaryChannel}, m_CWGpio{_CWGpio},
    m_CWPin{_CWPin}, m_CCWGpio{_CCWGpio}, m_CCWPin{_CCWPin}, m_ENGpio{_ENGpio},
    m_ENPin{_ENPin}
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
  if(m_ENGpio != NULL)
  {
    palSetPadMode(m_ENGpio, m_ENPin, PAL_MODE_OUTPUT_OPENDRAIN);
    palSetPad(m_ENGpio, m_ENPin);
  }
}

void MonsterShield::brake()
{
  stop();
  // Put the H-Bridge in brake mode
  palClearPad(m_CWGpio, m_CWPin);
  palClearPad(m_CCWGpio, m_CCWPin);
}
