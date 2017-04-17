#pragma once

#include <array>

#include "hal.h"

#define ADC_BUFFER_DEPTH 16
#define ADC_CHANNELS 2

class AdcTimer
{
public:
  AdcTimer(ADCDriver* _adcDriver, GPTDriver* _gptDriver,
           const uint32_t _channel1, const uint32_t _channel2,
           const uint32_t _timerFrequency, const uint32_t _timerPeriod);
  ADCConversionGroup& getAdcConversionGroup();
  GPTConfig& getGptConfig();
  void begin();
  std::array<adcsample_t, ADC_CHANNELS * ADC_BUFFER_DEPTH> getSamples() const;
  void adcCb(ADCDriver* _adcd, adcsample_t* _buffer, size_t _n);
  uint16_t value1() const;
  uint16_t value2() const;

protected:
  ADCDriver* m_adcDriver;
  GPTDriver* m_gptDriver;
  ADCConversionGroup m_adcConversionGroup;
  const uint32_t m_timerPeriod;
  GPTConfig m_gptConfig;
  std::array<adcsample_t, ADC_CHANNELS * ADC_BUFFER_DEPTH> m_samples;
  uint16_t m_currentValue1;
  uint16_t m_currentValue2;
};
