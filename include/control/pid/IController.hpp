#pragma once

#include <algorithm>

class IController {
public:
    IController(float kI, float minInt, float maxInt)
        : m_kI{ kI }, m_minInt{ minInt }, m_maxInt{ maxInt } {}

    float update(float setpoint, float measurement, float dt) {
        float const error = setpoint - measurement;
        
        m_integrator += 0.5f * m_kI * dt * (error + m_prevError);
        m_integrator = std::clamp(m_integrator, m_minInt, m_maxInt);
        m_prevError = error;

        return m_integrator;
    }

private:
    float m_kI{};
    float m_minInt{};
    float m_maxInt{};

    float m_integrator = 0.0f;
    float m_prevError = 0.0f;
};
