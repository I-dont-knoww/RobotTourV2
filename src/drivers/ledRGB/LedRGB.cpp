#include "drivers/LedRGB.hpp"

#include "Constants.hpp"

#include "state/Vector.hpp"

#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"

#include <cstdint>

LedRGB::LedRGB(uint redPin, uint greenPin, uint bluePin)
    : m_redChannel{ pwm_gpio_to_channel(redPin) },
      m_greenChannel{ pwm_gpio_to_channel(greenPin) },
      m_blueChannel{ pwm_gpio_to_channel(bluePin) },
      m_redSlice{ pwm_gpio_to_slice_num(redPin) },
      m_greenSlice{ pwm_gpio_to_slice_num(greenPin) },
      m_blueSlice{ pwm_gpio_to_slice_num(bluePin) } {
    gpio_set_function(redPin, GPIO_FUNC_PWM);
    gpio_set_function(greenPin, GPIO_FUNC_PWM);
    gpio_set_function(bluePin, GPIO_FUNC_PWM);

    gpio_set_drive_strength(redPin, GPIO_DRIVE_STRENGTH_12MA);
    gpio_set_drive_strength(greenPin, GPIO_DRIVE_STRENGTH_12MA);
    gpio_set_drive_strength(bluePin, GPIO_DRIVE_STRENGTH_12MA);

    auto pwmConfig = pwm_get_default_config();
    pwm_config_set_clkdiv(&pwmConfig, static_cast<float>(clock_get_hz(clk_sys)) /
                                          Drivers::LedRGB::TARGET_CLK_FREQUENCY);
    pwm_config_set_wrap(&pwmConfig, Drivers::LedRGB::PWM_WRAP);

    pwm_init(m_redSlice, &pwmConfig, true);
    pwm_init(m_greenSlice, &pwmConfig, true);
    pwm_init(m_blueSlice, &pwmConfig, true);

    setRGB({ 0.0f, 0.0f, 0.0f });
}

void LedRGB::setRGB(Vec3 const& color) {
    uint16_t const redLevel =
        static_cast<uint16_t>(color.x * static_cast<float>(Drivers::LedRGB::MAX_BRIGHTNESS));
    uint16_t const greenLevel =
        static_cast<uint16_t>(color.y * static_cast<float>(Drivers::LedRGB::MAX_BRIGHTNESS));
    uint16_t const blueLevel =
        static_cast<uint16_t>(color.z * static_cast<float>(Drivers::LedRGB::MAX_BRIGHTNESS));

    pwm_set_chan_level(m_redSlice, m_redChannel, Drivers::LedRGB::MAX_POWER - redLevel);
    pwm_set_chan_level(m_greenSlice, m_greenChannel, Drivers::LedRGB::MAX_POWER - greenLevel);
    pwm_set_chan_level(m_blueSlice, m_blueChannel, Drivers::LedRGB::MAX_POWER - blueLevel);
}
