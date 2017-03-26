#pragma once

#include "hal.h"

class Input
{
public:
    Input(stm32_gpio_t* _port, uint32_t _pin, uint16_t _mode = PAL_MODE_INPUT);
    void begin();
    bool read();

protected:
  stm32_gpio_t* m_port;
  const uint32_t m_pin;
  const uint16_t m_mode;
};
