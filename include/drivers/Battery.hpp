#pragma once

#include <cstdint>

using uint = unsigned int;

class Battery {
public:
    Battery(uint voltagePin);

    Battery(Battery const&) = delete;
    Battery& operator=(Battery const&) = delete;
    Battery(Battery&&) = delete;
    Battery& operator=(Battery&&) = delete;

    float voltage() { return m_voltage; }

private:
    float m_voltage{};
    uint16_t volatile m_temp{};
};
