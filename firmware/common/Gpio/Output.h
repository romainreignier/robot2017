#pragma once

#include "hal.h"

class Output
{
public:
  Output(stm32_gpio_t* _port, uint32_t _pin,
         uint16_t _mode = PAL_MODE_OUTPUT_PUSHPULL);
  void begin();
  void set();
  void clear();

protected:
  stm32_gpio_t* m_port;
  const uint32_t m_pin;
  const uint16_t m_mode;
};
