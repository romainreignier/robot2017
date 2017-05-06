#include "Output.h"

Output::Output(stm32_gpio_t *_port, uint32_t _pin, uint16_t _mode)
    : m_port{_port}, m_pin{_pin}, m_mode{_mode}
{
}

void Output::begin()
{
    palSetPadMode(m_port, m_pin, m_mode);
    palClearPad(m_port, m_pin);
}

void Output::set()
{
    palSetPad(m_port, m_pin);
}

void Output::clear()
{
    palClearPad(m_port, m_pin);
}
