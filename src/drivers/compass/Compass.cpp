#include "drivers/Compass.hpp"

#include "Compass.pio.h"

#include "Constants.hpp"

#include "hardware/dma.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "hardware/pio.h"

#include "pico/time.h"

#include <cmath>
#include <cstdint>
#include <iterator>

Compass::Compass(Vec3 const& down, uint sdaPin, uint sclPin)
    : Compass{ std::atan2f(down.y, down.z), std::atan2f(-down.x, Vec2{ down.y, down.z }.length()),
               sdaPin, sclPin } {}

Compass::Compass(float roll, float pitch, uint sdaPin, uint sclPin)
    : m_rawDataPtr{ reinterpret_cast<uint32_t>(&m_rawData[0]) },
      m_tiltCompA{ std::cosf(pitch) },
      m_tiltCompB{ std::sinf(roll) * std::sinf(pitch) },
      m_tiltCompC{ -std::cosf(roll) * std::sinf(pitch) },
      m_tiltCompD{ std::cosf(roll) },
      m_tiltCompE{ std::sinf(roll) },
      m_i2c{ i2c_get_instance((sclPin / 2) % 2) } {
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
    writeRegister(Register::CONTROL2, 0b10000000);
    sleep_ms(20);
    writeRegister(Register::CONTROL2, 0b00000000);
    sleep_ms(20);

    writeRegister(Register::CONTROL2, 0b00001000);
    writeRegister(Register::CONTROL1, 0b01100011);
    sleep_ms(500);

    // yeah idk it sometimes crashes and without this it
    // never resets properly or smth and the pio cant read
    // the data registers instead they just output zero or smth
    readRegister(Register::CONTROL2);
}

void Compass::endSetup() const { i2c_deinit(m_i2c); }

void Compass::setupContinuousRead(PIO pio, uint sdaPin, uint sclPin) const {
    uint sm = pio_claim_unused_sm(pio, true);

    uint offset = pio_add_program(pio, &compass_program);
    compass_program_init(pio, sm, offset, sdaPin, sclPin);

    uint const dmaReadDataChannel = dma_claim_unused_channel(true);
    uint const dmaReadControlChannel = dma_claim_unused_channel(true);

    auto dmaReadDataConfig = dma_channel_get_default_config(dmaReadDataChannel);
    channel_config_set_chain_to(&dmaReadDataConfig, dmaReadControlChannel);
    channel_config_set_read_increment(&dmaReadDataConfig, false);
    channel_config_set_write_increment(&dmaReadDataConfig, true);
    channel_config_set_dreq(&dmaReadDataConfig, pio_get_dreq(pio, sm, false));

    auto dmaReadControlConfig = dma_channel_get_default_config(dmaReadControlChannel);
    channel_config_set_read_increment(&dmaReadControlConfig, false);
    channel_config_set_write_increment(&dmaReadControlConfig, false);

    dma_channel_configure(dmaReadDataChannel, &dmaReadDataConfig, nullptr, &pio->rxf[sm],
                          dma_encode_transfer_count(3u), false);
    dma_channel_configure(dmaReadControlChannel, &dmaReadControlConfig,
                          &dma_hw->ch[dmaReadDataChannel].al2_write_addr_trig, &m_rawDataPtr,
                          dma_encode_transfer_count(1u), true);

    uint const dmaWriteDataChannel = dma_claim_unused_channel(true);
    uint const dmaWriteControlChannel = dma_claim_unused_channel(true);

    auto dmaWriteDataConfig = dma_channel_get_default_config(dmaWriteDataChannel);
    channel_config_set_chain_to(&dmaWriteDataConfig, dmaWriteControlChannel);
    channel_config_set_read_increment(&dmaWriteDataConfig, true);
    channel_config_set_write_increment(&dmaWriteDataConfig, false);
    channel_config_set_dreq(&dmaWriteDataConfig, pio_get_dreq(pio, sm, true));

    auto dmaWriteControlConfig = dma_channel_get_default_config(dmaWriteControlChannel);
    channel_config_set_read_increment(&dmaWriteControlConfig, false);
    channel_config_set_write_increment(&dmaWriteControlConfig, false);

    uint32_t const slaveAddressWithACK = (Drivers::Compass::I2C_ADDRESS << 2u) | 0b000000001;
    static uint32_t const s_compassAddresses[]{
        slaveAddressWithACK << 23u,
        (static_cast<uint32_t>(Register::DATA_OUT_X_MSB) << 1u | 0b000000001) << 23u,
        (slaveAddressWithACK | 0b000000010) << 23u,
        slaveAddressWithACK << 23u,
        (static_cast<uint32_t>(Register::DATA_OUT_X_LSB) << 1u | 0b000000001) << 23u,
        (slaveAddressWithACK | 0b000000010) << 23u,
        slaveAddressWithACK << 23u,
        (static_cast<uint32_t>(Register::DATA_OUT_Y_MSB) << 1u | 0b000000001) << 23u,
        (slaveAddressWithACK | 0b000000010) << 23u,
        slaveAddressWithACK << 23u,
        (static_cast<uint32_t>(Register::DATA_OUT_Y_LSB) << 1u | 0b000000001) << 23u,
        (slaveAddressWithACK | 0b000000010) << 23u,
        slaveAddressWithACK << 23u,
        (static_cast<uint32_t>(Register::DATA_OUT_Z_MSB) << 1u | 0b000000001) << 23u,
        (slaveAddressWithACK | 0b000000010) << 23u,
        slaveAddressWithACK << 23u,
        (static_cast<uint32_t>(Register::DATA_OUT_Z_LSB) << 1u | 0b000000001) << 23u,
        (slaveAddressWithACK | 0b000000010) << 23u
    };
    static uint32_t const s_compassAddressPointer =
        reinterpret_cast<uint32_t>(&s_compassAddresses[0]);

    dma_channel_configure(dmaWriteDataChannel, &dmaWriteDataConfig, &pio->txf[sm], nullptr,
                          dma_encode_transfer_count(std::size(s_compassAddresses)), false);
    dma_channel_configure(dmaWriteControlChannel, &dmaWriteControlConfig,
                          &dma_hw->ch[dmaWriteDataChannel].al3_read_addr_trig,
                          &s_compassAddressPointer, dma_encode_transfer_count(1u), true);
}

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
