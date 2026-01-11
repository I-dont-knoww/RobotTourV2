#pragma once

#include "Constants.hpp"

#include "state/Vector.hpp"

#include <cstdint>
#include <utility>

using uint = unsigned int;

class Motors {
public:
    class Controller {
    public:
        Controller(uint in1Pin, uint in2Pin);

        void spin(int power);

        float power() const {
            return static_cast<float>(m_power) / static_cast<float>(Drivers::Motors::MAX_POWER);
        }

    private:
        uint const m_in1Channel{};
        uint const m_in2Channel{};

        uint const m_in1Slice{};
        uint const m_in2Slice{};

        int m_power{};
    };

    class CurrentSensor {
    public:
        CurrentSensor(uint leftCurrentPin, uint rightCurrentPin);

        CurrentSensor(CurrentSensor const&) = delete;
        CurrentSensor& operator=(CurrentSensor const&) = delete;
        CurrentSensor(CurrentSensor&&) = delete;
        CurrentSensor& operator=(CurrentSensor&&) = delete;

        Vec2 data() const;

    private:
        float getBias(uint currentPin);
        void setupRead(uint leftCurrentPin, uint rightCurrentPin);

        uint16_t volatile m_rawData[2]{ 0u, 0u };
        Vec2 m_bias{};
    };

    Motors(uint leftIn1Pin, uint leftIn2Pin, uint rightIn1Pin, uint rightIn2Pin,
           uint leftCurrentPin, uint rightCurrentPin);

    constexpr decltype(auto) left(this auto&& self) {
        return std::forward_like<decltype(self)>(self.m_left);
    }
    constexpr decltype(auto) right(this auto&& self) {
        return std::forward_like<decltype(self)>(self.m_right);
    }

    void spin(int power) { spin(power, power); }
    void spin(int leftPower, int rightPower) {
        left().spin(-leftPower);
        right().spin(-rightPower);
    }

    Vec2 power() const { return { m_left.power(), m_right.power() }; }
    Vec2 current() const { return m_currentSensor.data(); }

private:
    Controller m_left;
    Controller m_right;

    CurrentSensor m_currentSensor;
};
