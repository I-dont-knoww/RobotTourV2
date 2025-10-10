// https://github.com/finani/ICM42688

#pragma once

#include "Constants.hpp"

#include "state/Vector.hpp"

#include "hardware/gpio.h"
#include "hardware/pio.h"
#include "hardware/spi.h"

#include <bit>
#include <cstddef>
#include <type_traits>

class Gyroscope {
public:
    Gyroscope(PIO pio, uint csPin, uint sckPin, uint misoPin, uint mosiPin, uint intPin);

    Gyroscope(Gyroscope const&) = delete;
    Gyroscope& operator=(Gyroscope const&) = delete;
    Gyroscope(Gyroscope&&) = delete;
    Gyroscope& operator=(Gyroscope&&) = delete;

    float angularVelocity() const {
        uint16_t x = static_cast<uint16_t>(m_rawData[0]);
        uint16_t y = static_cast<uint16_t>(m_rawData[1]);
        uint16_t z = static_cast<uint16_t>(m_rawData[2]);

        Vec3 const direction{ static_cast<float>(std::bit_cast<int16_t>(x)),
                              static_cast<float>(std::bit_cast<int16_t>(y)),
                              static_cast<float>(std::bit_cast<int16_t>(z)) };

        return Vec3::dot(direction / Drivers::Gyroscope::RESOLUTION - m_bias, m_down);
    }

    Vec3 const& down() const { return m_down; }

private:
    static constexpr uint8_t REG_BANK_SEL = 0x76;
    enum class Bank0 : uint8_t {
        DEVICE_CONFIG = 0x11,
        DRIVE_CONFIG = 0x13,
        INT_CONFIG = 0x14,
        FIFO_CONFIG = 0x16,
        TEMP_DATA1 = 0x1d,
        TEMP_DATA0 = 0x1e,
        ACCEL_DATA_X1 = 0x1f,
        ACCEL_DATA_X0 = 0x20,
        ACCEL_DATA_Y1 = 0x21,
        ACCEL_DATA_Y0 = 0x22,
        ACCEL_DATA_Z1 = 0x23,
        ACCEL_DATA_Z0 = 0x24,
        GYRO_DATA_X1 = 0x25,
        GYRO_DATA_X0 = 0x26,
        GYRO_DATA_Y1 = 0x27,
        GYRO_DATA_Y0 = 0x28,
        GYRO_DATA_Z1 = 0x29,
        GYRO_DATA_Z0 = 0x2a,
        TMST_FSYNCH = 0x2b,
        TMST_FSYNCL = 0x2c,
        INT_STATUS = 0x2d,
        FIFO_COUNTH = 0x2e,
        FIFO_COUNTL = 0x2f,
        FIFO_DATA = 0x30,
        APEX_DATA0 = 0x31,
        APEX_DATA1 = 0x32,
        APEX_DATA2 = 0x33,
        APEX_DATA3 = 0x34,
        APEX_DATA4 = 0x35,
        APEX_DATA5 = 0x36,
        INT_STATUS2 = 0x37,
        INT_STATUS3 = 0x38,
        SIGNAL_PATH_RESET = 0x4b,
        INTF_CONFIG0 = 0x4c,
        INTF_CONFIG1 = 0x4d,
        PWR_MGMT0 = 0x4e,
        GYRO_CONFIG0 = 0x4f,
        ACCEL_CONFIG0 = 0x50,
        GYRO_CONFIG1 = 0x51,
        GYRO_ACCEL_CONFIG0 = 0x52,
        ACCEL_CONFIG1 = 0x53,
        TMST_CONFIG = 0x54,
        APEX_CONFIG0 = 0x56,
        SMD_CONFIG = 0x57,
        FIFO_CONFIG1 = 0x5f,
        FIFO_CONFIG2 = 0x60,
        FIFO_CONFIG3 = 0x61,
        FSYNC_CONFIG = 0x62,
        INT_CONFIG0 = 0x63,
        INT_CONFIG1 = 0x64,
        INT_SOURCE0 = 0x65,
        INT_SOURCE1 = 0x66,
        INT_SOURCE3 = 0x68,
        INT_SOURCE4 = 0x69,
        FIFO_LOST_PKT0 = 0x6c,
        FIFO_LOST_PKT1 = 0x6d,
        SELF_TEST_CONFIG = 0x70,
        WHO_AM_I = 0x75
    };
    enum class Bank1 : uint8_t {
        SENSOR_CONFIG0 = 0x03,
        GYRO_CONFIG_STATIC2 = 0x0b,
        GYRO_CONFIG_STATIC3 = 0x0c,
        GYRO_CONFIG_STATIC4 = 0x0d,
        GYRO_CONFIG_STATIC5 = 0x0e,
        GYRO_CONFIG_STATIC6 = 0x0f,
        GYRO_CONFIG_STATIC7 = 0x10,
        GYRO_CONFIG_STATIC8 = 0x11,
        GYRO_CONFIG_STATIC9 = 0x12,
        GYRO_CONFIG_STATIC10 = 0x13,
        XG_ST_DATA = 0x5f,
        YG_ST_DATA = 0x60,
        ZG_ST_DATA = 0x61,
        TMSTVAL0 = 0x62,
        TMSTVAL1 = 0x63,
        TMSTVAL2 = 0x64,
        INTF_CONFIG4 = 0x7a,
        INTF_CONFIG5 = 0x7b,
        INTF_CONFIG6 = 0x7c
    };
    enum class Bank2 : uint8_t {
        ACCEL_CONFIG_STATIC2 = 0x03,
        ACCEL_CONFIG_STATIC3 = 0x04,
        ACCEL_CONFIG_STATIC4 = 0x05,
        XA_ST_DATA = 0x3b,
        YA_ST_DATA = 0x3c,
        ZA_ST_DATA = 0x3d
    };
    enum class Bank3 : uint8_t { CLKDIV = 0x2a };
    enum class Bank4 : uint8_t {
        APEX_CONFIG1 = 0x40,
        APEX_CONFIG2 = 0x41,
        APEX_CONFIG3 = 0x42,
        APEX_CONFIG4 = 0x43,
        APEX_CONFIG5 = 0x44,
        APEX_CONFIG6 = 0x45,
        APEX_CONFIG7 = 0x46,
        APEX_CONFIG8 = 0x47,
        APEX_CONFIG9 = 0x48,
        ACCEL_WOM_X_THR = 0x4a,
        ACCEL_WOM_Y_THR = 0x4b,
        ACCEL_WOM_Z_THR = 0x4c,
        INT_SOURCE6 = 0x4d,
        INT_SOURCE7 = 0x4e,
        INT_SOURCE8 = 0x4f,
        INT_SOURCE9 = 0x50,
        INT_SOURCE10 = 0x51,
        OFFSET_USER0 = 0x77,
        OFFSET_USER1 = 0x78,
        OFFSET_USER2 = 0x79,
        OFFSET_USER3 = 0x7a,
        OFFSET_USER4 = 0x7b,
        OFFSET_USER5 = 0x7c,
        OFFSET_USER6 = 0x7d,
        OFFSET_USER7 = 0x7e,
        OFFSET_USER8 = 0x7f,
    };

    spi_inst_t* setupRegisterReadWrite(uint csPin, uint sckPin, uint misoPin, uint mosiPin,
                                       uint intPin) const;
    void setupRegisters(spi_inst_t* spi, uint csPin) const;

    void getBias(spi_inst_t* spi, uint csPin);
    void getDown(spi_inst_t* spi, uint csPin);

    void endSetup(spi_inst_t* spi) const;
    void setupPIORead(PIO pio, uint csPin, uint sckPin, uint misoPin, uint mosiPin, uint intPin);

    template <typename TBank>
    void selectBank(spi_inst_t* spi, uint csPin, TBank reg) const {
        uint8_t bank{};

        if constexpr (std::is_same_v<TBank, Bank0>) bank = 0u;
        else if constexpr (std::is_same_v<TBank, Bank1>) bank = 1u;
        else if constexpr (std::is_same_v<TBank, Bank2>) bank = 2u;
        else if constexpr (std::is_same_v<TBank, Bank3>) bank = 3u;
        else if constexpr (std::is_same_v<TBank, Bank4>) bank = 4u;
        else static_assert(false);

        gpio_put(csPin, false);
        spi_write_blocking(spi, &REG_BANK_SEL, 1u);
        spi_write_blocking(spi, &bank, 1u);
        gpio_put(csPin, true);
    }

    template <typename T>
    void readRegisters(spi_inst_t* spi, uint csPin, T startReg, uint8_t* data,
                       size_t length) const {
        selectBank(spi, csPin, startReg);

        uint8_t const registerValue = (0b10000000) | static_cast<uint8_t>(startReg);

        gpio_put(csPin, false);
        spi_write_blocking(spi, &registerValue, 1u);
        spi_read_blocking(spi, 0u, data, length);
        gpio_put(csPin, true);
    }

    template <typename T>
    uint8_t readRegister(spi_inst_t* spi, uint csPin, T reg) const {
        uint8_t data{};
        readRegisters(spi, csPin, reg, &data, 1u);
        return data;
    }

    template <typename T>
    void writeRegister(spi_inst_t* spi, uint csPin, T reg, uint8_t value) const {
        selectBank(spi, csPin, reg);

        uint8_t const registerValue = static_cast<uint8_t>(reg);

        gpio_put(csPin, false);
        spi_write_blocking(spi, &registerValue, 1u);
        spi_write_blocking(spi, &value, 1u);
        gpio_put(csPin, true);
    }

    Vec3 m_down{};
    Vec3 m_bias{};

    uint32_t volatile m_rawData[3]{ 0u, 0u, 0u };
    uint32_t const m_rawDataPtr{};
};
