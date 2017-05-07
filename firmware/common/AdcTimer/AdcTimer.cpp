#include "AdcTimer.h"

AdcTimer::AdcTimer(ADCDriver* _adcDriver, GPTDriver* _gptDriver,
                   const uint32_t _timerPeriod)
  : m_adcDriver{_adcDriver}, m_gptDriver{_gptDriver},
    m_timerPeriod{_timerPeriod}, m_currentValue1{0}, m_currentValue2{0}
{
}

void AdcTimer::begin(const GPTConfig* _gptConfig,
                     const ADCConversionGroup* _adcConversionGroup)
{
  // Activate the GPT driver
  gptStart(m_gptDriver, _gptConfig);

  // Activates the ADC driver
  adcStart(m_adcDriver, NULL);

  // Starts ADC conversion triggered by the timer
  adcStartConversion(
    m_adcDriver, _adcConversionGroup, m_samples.data(), ADC_BUFFER_DEPTH);
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
