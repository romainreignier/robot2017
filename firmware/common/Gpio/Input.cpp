#include "Input.h"

Input::Input(stm32_gpio_t *_port, uint32_t _pin, uint16_t _mode)
    : m_port{_port}, m_pin{_pin}, m_mode{_mode}
{
}

void Input::begin()
{
    palSetPadMode(m_port, m_pin, m_mode);
}

bool Input::read()
{
    return (palReadPad(m_port, m_pin)) ? true : false;
}
