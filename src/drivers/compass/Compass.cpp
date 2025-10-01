#include "drivers/Compass.hpp"

#include "Constants.hpp"

#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "hardware/pio.h"

#include <cmath>
#include <cstdint>

#include <cstdio>

Compass::Compass(Vec3 const& down, uint sdaPin, uint sclPin)
    : Compass{ std::atan2f(down.y, down.z), std::atan2f(-down.x, Vec2{ down.y, down.z }.length()),
               sdaPin, sclPin } {}

Compass::Compass(float roll, float pitch, uint sdaPin, uint sclPin)
    : m_i2c{ i2c_get_instance((sclPin / 2) % 2) },
      m_tiltCompA{ std::cosf(pitch) },
      m_tiltCompB{ std::sinf(roll) * std::sinf(pitch) },
      m_tiltCompC{ -std::cosf(roll) * std::sinf(pitch) },
      m_tiltCompD{ std::cosf(roll) },
      m_tiltCompE{ std::sinf(roll) } {
    setupRegisterReadWrite(sdaPin, sclPin);
    setupRegisters();
}

void Compass::setupRegisterReadWrite(uint sdaPin, uint sclPin) const {
    i2c_init(m_i2c, 400'000);

    gpio_set_function(sdaPin, GPIO_FUNC_I2C);
    gpio_set_function(sclPin, GPIO_FUNC_I2C);
    gpio_pull_up(sdaPin);
    gpio_pull_up(sclPin);
}

void Compass::setupRegisters() const {
    writeRegister(Register::CONTROL2, 0b00001000);
    writeRegister(Register::CONTROL1, 0b01100011);
}

void Compass::endSetup() const { i2c_deinit(m_i2c); }

void Compass::setupPIORead(PIO pio, uint sdaPin, uint sclPin) const {}

uint8_t Compass::readRegister(Register reg) const {
    uint8_t const regValue = static_cast<uint8_t>(reg);
    i2c_write_blocking(m_i2c, Drivers::Compass::I2C_ADDRESS, &regValue, 1u, true);

    uint8_t data{};
    i2c_read_blocking(m_i2c, Drivers::Compass::I2C_ADDRESS, &data, 1u, false);

    return data;
}

void Compass::writeRegister(Register reg, uint8_t value) const {
    uint8_t const data[2]{ static_cast<uint8_t>(reg), value };
    i2c_write_blocking(m_i2c, Drivers::Compass::I2C_ADDRESS, data, 2u, true);
}
