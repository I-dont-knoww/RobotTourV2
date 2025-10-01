#include "drivers/Button.hpp"

#include "hardware/gpio.h"

Button::Button(uint pin) : m_pin{ pin } {
    gpio_init(m_pin);
    gpio_set_dir(m_pin, GPIO_IN);
    gpio_pull_up(m_pin);
}

bool Button::isPressed() const { return !gpio_get(m_pin); }

void Button::waitForPress() const { while (!isPressed()); }
void Button::waitForUnpress() const { while (isPressed()); }

void Button::waitForClick() const { waitForUnpress(); waitForPress(); waitForUnpress(); };
