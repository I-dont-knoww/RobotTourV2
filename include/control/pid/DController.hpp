#pragma once

#include "filters/LagFilter.hpp"

#include "Constants.hpp"

class DController {
public:
    DController(float kD, float alpha) : m_filter{ alpha }, m_kD{ kD } {}

    float update(float, float measurement, float dt) {
        // m_differentiator = -(2.0f * m_kD * (measurement - m_prevMeasurement) -
        //                      (2.0f * m_tau - dt) * m_differentiator) /
        //                    (2.0f * m_tau + dt);
        // m_prevMeasurement = measurement;

        // return m_differentiator;

        float const filteredMeasurement = m_filter.update(measurement, dt);

        float const output = -m_kD * (filteredMeasurement - m_prevMeasurement) / dt;
        m_prevMeasurement = filteredMeasurement;
        return output;
    }

private:
    LagFilter m_filter;

    float m_kD{};
    // float m_tau{};

    float m_differentiator = 0.0f;
    float m_prevMeasurement = 0.0f;
};
