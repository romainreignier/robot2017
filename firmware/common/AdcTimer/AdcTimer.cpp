#include "AdcTimer.h"

AdcTimer::AdcTimer(ADCDriver* _adcDriver, GPTDriver* _gptDriver,
                   const uint32_t _channel1, const uint32_t _channel2,
                   const uint32_t _timerFrequency, const uint32_t _timerPeriod)
  : m_adcDriver{_adcDriver}, m_gptDriver{_gptDriver},
    m_adcConversionGroup{
      TRUE,
      ADC_CHANNELS,
      NULL,
      NULL,
      ADC_CFGR_EXTEN_RISING | ADC_CFGR_EXTSEL_SRC(13),
      ADC_TR(0, 4095),
      {0, 0},
      {ADC_SQR1_SQ1_N(_channel1) | ADC_SQR1_SQ2_N(_channel2), 0, 0, 0}},
    m_timerPeriod{_timerPeriod}, m_gptConfig{.frequency = _timerFrequency,
                                             .callback = NULL,
                                             .cr2 = TIM_CR2_MMS_1,
                                             .dier = 0U},
    m_currentValue1{0}, m_currentValue2{0}
{
}

ADCConversionGroup& AdcTimer::getAdcConversionGroup()
{
  return m_adcConversionGroup;
}

GPTConfig& AdcTimer::getGptConfig()
{
  return m_gptConfig;
}

void AdcTimer::begin()
{
  // Activate the GPT driver
  gptStart(m_gptDriver, &m_gptConfig);

  // Activates the ADC driver
  adcStart(m_adcDriver, NULL);

  // Starts ADC conversion triggered by the timer
  adcStartConversion(
    m_adcDriver, &m_adcConversionGroup, m_samples.data(), ADC_BUFFER_DEPTH);
  gptStartContinuous(m_gptDriver, m_timerPeriod);
}

std::array<adcsample_t, ADC_CHANNELS * ADC_BUFFER_DEPTH>
AdcTimer::getSamples() const
{
  return m_samples;
}

void AdcTimer::adcCb(ADCDriver* _adcd, adcsample_t* _buffer, size_t _n)
{
  (void)_n;
  if(_adcd == m_adcDriver)
  {
    uint32_t sum1 = 0;
    uint32_t sum2 = 0;
    for(size_t i = 0; i < m_samples.size() / 2; i += 2)
    {
      sum1 += _buffer[i];
      sum2 += _buffer[i + 1];
    }
    sum1 /= ADC_BUFFER_DEPTH / 2;
    sum2 /= ADC_BUFFER_DEPTH / 2;
    m_currentValue1 = sum1;
    m_currentValue2 = sum2;
  }
}

uint16_t AdcTimer::value1() const
{
  return m_currentValue1;
}

uint16_t AdcTimer::value2() const
{
  return m_currentValue2;
}
