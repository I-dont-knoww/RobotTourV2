#pragma once

#include <cmath>

class SController {
public:
    SController(float kS) : m_kS{ kS } {}

    float update(float setpoint, float) {
        return (setpoint != 0.0f) * std::copysignf(m_kS, setpoint);
    }

private:
    float m_kS{};
};
