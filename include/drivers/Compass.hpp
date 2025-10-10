// https://github.com/pcbcupid/PCBCUPID-QMC6309

#pragma once

#include "state/Vector.hpp"

#include "hardware/i2c.h"
#include "hardware/pio.h"

#include <algorithm>
#include <bit>
#include <cmath>
#include <concepts>
#include <cstdint>

class Compass {
public:
    struct Calibration {
        Vec3 scale{};
        Vec3 offset{};
    };

    Compass(Vec3 const& down, uint sdaPin, uint sclPin);

    template <typename F>
        requires requires(F f, Vec3 const& sample, Vec3 const& minimums, Vec3 const& maximums,
                          int sampleCount) {
            { f(sample, minimums, maximums, sampleCount) } -> std::same_as<bool>;
        }
    Calibration getCalibration(F const& stopCondition) const {
        uint8_t const startRegister = static_cast<uint8_t>(Register::DATA_OUT_X_LSB);
        uint8_t const endRegister = static_cast<uint8_t>(Register::DATA_OUT_Z_MSB);

        Vec3 sample{};

        Vec3 minimums{ 0.0f, 0.0f, 0.0f };
        Vec3 maximums{ 0.0f, 0.0f, 0.0f };

        int sampleCount = 0;

        while (!stopCondition(sample, minimums, maximums, ++sampleCount)) {
            uint8_t registerValues[6]{};

            for (uint8_t reg = startRegister; reg <= endRegister; ++reg)
                registerValues[reg - startRegister] = readRegister(static_cast<Register>(reg));

            uint16_t x = registerValues[1] << 8u | registerValues[0];
            uint16_t y = registerValues[3] << 8u | registerValues[2];
            uint16_t z = registerValues[5] << 8u | registerValues[4];

            sample = { static_cast<float>(std::bit_cast<int16_t>(x)),
                       static_cast<float>(std::bit_cast<int16_t>(y)),
                       static_cast<float>(std::bit_cast<int16_t>(z)) };

            minimums.x = std::min(sample.x, minimums.x);
            minimums.y = std::min(sample.y, minimums.y);
            minimums.z = std::min(sample.z, minimums.z);

            maximums.x = std::max(sample.x, maximums.x);
            maximums.y = std::max(sample.y, maximums.y);
            maximums.z = std::max(sample.z, maximums.z);
        }

        Vec3 chords = (maximums - minimums) / 2.0f;
        float chordAverage = (chords.x + chords.y + chords.z) / 3.0f;

        Vec3 scale{ chordAverage / chords.x, chordAverage / chords.y, chordAverage / chords.z };
        Vec3 offset = (minimums + maximums) / 2.0f;

        return { scale, offset };
    }
    void setCalibration(Calibration const& calibration) { m_calibration = calibration; }

    void switchToPIO(PIO pio, uint sdaPin, uint sclPin) const {
        endSetup();
        setupContinuousRead(pio, sdaPin, sclPin);
    }

    float getHeading() {
        uint16_t x = static_cast<uint16_t>(m_rawData[0]);
        uint16_t y = static_cast<uint16_t>(m_rawData[1]);
        uint16_t z = static_cast<uint16_t>(m_rawData[2]);

        Vec3 const rawData{ static_cast<float>(std::bit_cast<int16_t>(x)),
                            static_cast<float>(std::bit_cast<int16_t>(y)),
                            static_cast<float>(std::bit_cast<int16_t>(z)) };
        Vec3 const calibratedData = (rawData - m_calibration.offset) * m_calibration.scale;

        float const compensatedX = m_tiltCompA * calibratedData.x + m_tiltCompB * calibratedData.y +
                                   m_tiltCompC * calibratedData.z;
        float const compensatedY = m_tiltCompD * calibratedData.y + m_tiltCompE * calibratedData.z;
        float const heading = std::atan2f(compensatedY, compensatedX);

        return heading;
    }

private:
    Compass(float roll, float pitch, uint sda, uint scl);

    enum class Register : uint8_t {
        CHIP_ID = 0x00,
        DATA_OUT_X_LSB = 0x01,
        DATA_OUT_X_MSB = 0x02,
        DATA_OUT_Y_LSB = 0x03,
        DATA_OUT_Y_MSB = 0x04,
        DATA_OUT_Z_LSB = 0x05,
        DATA_OUT_Z_MSB = 0x06,
        STATUS = 0x09,
        CONTROL1 = 0x0a,
        CONTROL2 = 0x0b,
        SELF_TEST = 0x0e,
        DATA_OUT_X_SELF_TEST = 0x13,
        DATA_OUT_Y_SELF_TEST = 0x14,
        DATA_OUT_Z_SELF_TEST = 0x15
    };

    void setupRegisterReadWrite(uint sdaPin, uint sclPin) const;
    void setupRegisters() const;

    void endSetup() const;
    void setupContinuousRead(PIO pio, uint sdaPin, uint sclPin) const;

    uint8_t readRegister(Register reg) const;
    void writeRegister(Register reg, uint8_t value) const;

    Calibration m_calibration{};

    uint32_t volatile m_rawData[3]{ 0u, 0u, 0u };
    uint32_t const m_rawDataPtr{};

    float const m_tiltCompA{};
    float const m_tiltCompB{};
    float const m_tiltCompC{};
    float const m_tiltCompD{};
    float const m_tiltCompE{};

    i2c_inst_t* const m_i2c{};
};
