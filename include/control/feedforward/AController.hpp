#pragma once

#include "Constants.hpp"

class AController {
public:
    AController(float kA, float cutoff)
        : m_kA{ kA }, m_tau{ 1.0f / (2.0f * Constants::PI * cutoff) } {}

    float update(float setpoint, float, float dt) {
        m_differentiator =
            (2.0f * m_kA * (setpoint - m_prevSetpoint) + (2.0f * m_tau - dt) * m_differentiator) /
            (2.0f * m_tau + dt);
        m_prevSetpoint = setpoint;

        return m_differentiator;

        // float const output = m_kA * (setpoint - m_prevSetpoint) / dt;
        // m_prevSetpoint = setpoint;
        // return output;
    }

private:
    float m_kA{};
    float m_tau{};

    float m_differentiator = 0.0f;
    float m_prevSetpoint = 0.0f;
};
