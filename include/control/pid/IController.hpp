#pragma once

#include <algorithm>

class IController {
public:
    IController(float kI, float minInt, float maxInt, float dt)
        : m_k{ kI * dt }, m_minInt{ minInt }, m_maxInt{ maxInt } {}

    float update(float setpoint, float measurement) {
        float const error = setpoint - measurement;

        m_integrator += 0.5f * m_k * (error + m_prevError);
        m_integrator = std::clamp(m_integrator, m_minInt, m_maxInt);
        m_prevError = error;

        return m_integrator;
    }

private:
    float const m_k{};
    float const m_minInt{};
    float const m_maxInt{};

    float m_integrator = 0.0f;
    float m_prevError = 0.0f;
};
