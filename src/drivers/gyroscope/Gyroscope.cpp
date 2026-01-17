#include "drivers/Gyroscope.hpp"

#include "Gyroscope.pio.h"

#include "Constants.hpp"

#include "state/Vector.hpp"

#include "hardware/dma.h"
#include "hardware/gpio.h"
#include "hardware/pio.h"
#include "hardware/spi.h"

#include "pico/time.h"

#include <algorithm>
#include <bit>
#include <cstddef>

Gyroscope::Gyroscope(PIO pio, uint csPin, uint sckPin, uint misoPin, uint mosiPin, uint intPin)
    : m_rawDataPtr{ reinterpret_cast<uint32_t>(&m_rawData[0]) } {
    spi_inst_t* spi = setupRegisterReadWrite(csPin, sckPin, misoPin, mosiPin);
    setupRegisters(spi, csPin);

    getBias(spi, csPin);
    getDown(spi, csPin);

    endSetup(spi);
    setupPIORead(pio, csPin, sckPin, misoPin, mosiPin, intPin);
}

spi_inst_t* Gyroscope::setupRegisterReadWrite(uint csPin, uint sckPin, uint misoPin,
                                              uint mosiPin) const {
    spi_inst_t* spi = SPI_INSTANCE(sckPin == 10u || sckPin == 14u || sckPin == 26u);

    spi_init(spi, 24'000'000);
    spi_set_format(spi, 8, SPI_CPOL_1, SPI_CPHA_1, SPI_MSB_FIRST);

    gpio_set_function(sckPin, GPIO_FUNC_SPI);
    gpio_set_function(misoPin, GPIO_FUNC_SPI);
    gpio_set_function(mosiPin, GPIO_FUNC_SPI);

    gpio_init(csPin);
    gpio_set_dir(csPin, GPIO_OUT);
    gpio_put(csPin, true);

    return spi;
}

void Gyroscope::setupRegisters(spi_inst_t* spi, uint csPin) const {
    // reset the device
    writeRegister(spi, csPin, Bank0::DEVICE_CONFIG, 0b00000001);
    sleep_ms(1);

    // switch gyro and accel to low noise mode and disable temperature sensor
    writeRegister(spi, csPin, Bank0::PWR_MGMT0, 0b00101111);

    // switch accelerometer to 2g at 32kHz
    writeRegister(spi, csPin, Bank0::ACCEL_CONFIG0, 0b01100001);
    // switch gyro to 250 dps at 32kHz
    writeRegister(spi, csPin, Bank0::GYRO_CONFIG0, 0b00110001);

    // disable internal filters
    writeRegister(spi, csPin, Bank1::GYRO_CONFIG_STATIC2, 0b00000011);

    // switch int1 and int2 to pulsed mode active low push pull
    writeRegister(spi, csPin, Bank0::INT_CONFIG, 0b00010010);
    // enable proper int1 and int2 operation and disable deassertion duration
    writeRegister(spi, csPin, Bank0::INT_CONFIG1, 0b01110000);
    // route data ready interrupt to int1
    writeRegister(spi, csPin, Bank0::INT_SOURCE0, 0b00001000);

    sleep_ms(100);
}

void Gyroscope::getBias(spi_inst_t* spi, uint csPin) {
    m_bias = { 0.0f, 0.0f, 0.0f };
    for (size_t sample = 0; sample < Drivers::Gyroscope::GYRO_CALIBRATION_SAMPLE_COUNT; ++sample) {
        uint8_t rawMeasurements[6]{};
        readRegisters(spi, csPin, Bank0::GYRO_DATA_X1, rawMeasurements, 6u);

        uint16_t x = rawMeasurements[0] << 8u | rawMeasurements[1];
        uint16_t y = rawMeasurements[2] << 8u | rawMeasurements[3];
        uint16_t z = rawMeasurements[4] << 8u | rawMeasurements[5];

        m_bias += Vec3{ static_cast<float>(std::bit_cast<int16_t>(x)),
                        static_cast<float>(std::bit_cast<int16_t>(y)),
                        static_cast<float>(std::bit_cast<int16_t>(z)) };
    }
    m_bias /= Drivers::Gyroscope::RESOLUTION *
              static_cast<float>(Drivers::Gyroscope::GYRO_CALIBRATION_SAMPLE_COUNT);
}

void Gyroscope::getDown(spi_inst_t* spi, uint csPin) {
    m_down = { 0.0f, 0.0f, 0.0f };
    for (size_t sample = 0; sample < Drivers::Gyroscope::DOWN_DIRECTION_SAMPLE_COUNT; ++sample) {
        uint8_t rawMeasurements[6]{};
        readRegisters(spi, csPin, Bank0::ACCEL_DATA_X1, rawMeasurements, 6u);

        uint16_t x = rawMeasurements[0] << 8u | rawMeasurements[1];
        uint16_t y = rawMeasurements[2] << 8u | rawMeasurements[3];
        uint16_t z = rawMeasurements[4] << 8u | rawMeasurements[5];

        m_down += Vec3{ static_cast<float>(std::bit_cast<int16_t>(x)),
                        static_cast<float>(std::bit_cast<int16_t>(y)),
                        static_cast<float>(std::bit_cast<int16_t>(z)) };
    }
    m_down /= m_down.length();
}

void Gyroscope::endSetup(spi_inst_t* spi) const { spi_deinit(spi); }

void Gyroscope::setupPIORead(PIO pio, uint csPin, uint sckPin, uint misoPin, uint mosiPin,
                             uint intPin) {
    uint const sm = pio_claim_unused_sm(pio, true);

    uint const offset = pio_add_program(pio, &gyroscope_program);
    gyroscope_program_init(pio, sm, offset, csPin, sckPin, misoPin, mosiPin, intPin);

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

    uint const dmaWriteChannel = dma_claim_unused_channel(true);

    auto dmaWriteConfig = dma_channel_get_default_config(dmaWriteChannel);
    channel_config_set_read_increment(&dmaWriteConfig, false);
    channel_config_set_write_increment(&dmaWriteConfig, false);
    channel_config_set_dreq(&dmaWriteConfig, pio_get_dreq(pio, sm, true));

    static uint32_t const s_gyroDataAddress = static_cast<uint32_t>(Bank0::GYRO_DATA_X1) |
                                              0b10000000;
    dma_channel_configure(dmaWriteChannel, &dmaWriteConfig, &pio->txf[sm], &s_gyroDataAddress,
                          dma_encode_endless_transfer_count(), true);
}
