#pragma once

#include "filters/LagFilter.hpp"

#include "Constants.hpp"

class AController {
public:
    AController(float kA, float alpha, float dt)
        : m_filter{ alpha }, m_k{ kA / dt } {}

    float update(float setpoint, float) {
        float const filteredSetpoint = m_filter.update(setpoint);
        
        float const output = m_k * (filteredSetpoint - m_prevSetpoint);
        m_prevSetpoint = filteredSetpoint;
        return output;
    }

private:
    LagFilter m_filter;
    float const m_k{};

    float m_prevSetpoint = 0.0f;
};
