#include "L298.h"

L298::L298(PWMDriver* _driver, const uint32_t _timerFrequency,
           const uint32_t _timerPeriod, const uint8_t _channelA,
           const bool _isAComplementaryChannel, const uint8_t _channelB,
           const bool _isBComplementaryChannel, stm32_gpio_t* _inAGpio,
           const uint32_t _inAPin, stm32_gpio_t* _inBGpio,
           const uint32_t _inBPin)
  : Motor{_driver,
          _timerFrequency,
          _timerPeriod,
          _channelA,
          _isAComplementaryChannel},
    m_channelA(_channelA - 1),
    m_isAComplementaryChannel{_isAComplementaryChannel},
    m_channelB(_channelB - 1),
    m_isBComplementaryChannel{_isBComplementaryChannel}, m_inAGpio{_inAGpio},
    m_inAPin{_inAPin}, m_inBGpio{_inBGpio}, m_inBPin{_inBPin}
{
}

void L298::begin()
{
  m_pwmCfg.frequency = m_timerFrequency;
  m_pwmCfg.period = m_timerPeriod;
  m_pwmCfg.callback = NULL;
  m_pwmCfg.channels[m_channelA].mode =
    (m_isAComplementaryChannel ? PWM_COMPLEMENTARY_OUTPUT_ACTIVE_HIGH
                               : PWM_OUTPUT_ACTIVE_HIGH);
  m_pwmCfg.channels[m_channelB].mode =
    (m_isBComplementaryChannel ? PWM_COMPLEMENTARY_OUTPUT_ACTIVE_HIGH
                               : PWM_OUTPUT_ACTIVE_HIGH);
  setOutputPinsMode();
  pwmStart(m_driver, &m_pwmCfg);
  stop();
}

void L298::stop()
{
  pwmI(0);
}

void L298::brake()
{
  stop();
}

void L298::setOutputPinsMode()
{
}

void L298::changeDirection(Motor::eDirection _direction)
{
  // Never called, changing direction directly done in pwmI() because
  // reimplemented
  (void)_direction;
}

void L298::pwmI(int16_t _percentage)
{
  if(_percentage >= 0)
  {
    pwmEnableChannelI(
      m_driver,
      m_channelB,
      PWM_PERCENTAGE_TO_WIDTH(m_driver, static_cast<uint16_t>(0)));

    _percentage = (_percentage > 10000) ? 10000 : _percentage;
    pwmEnableChannelI(
      m_driver,
      m_channelA,
      PWM_PERCENTAGE_TO_WIDTH(m_driver, static_cast<uint16_t>(_percentage)));
  }
  else if(_percentage < 0)
  {
    pwmEnableChannelI(
      m_driver,
      m_channelA,
      PWM_PERCENTAGE_TO_WIDTH(m_driver, static_cast<uint16_t>(0)));

    _percentage *= -1;
    _percentage = (_percentage > 10000) ? 10000 : _percentage;
    pwmEnableChannelI(
      m_driver,
      m_channelB,
      PWM_PERCENTAGE_TO_WIDTH(m_driver, static_cast<uint16_t>(_percentage)));
  }
}
