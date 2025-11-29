#pragma once

#include "filters/LagFilter.hpp"

#include "Constants.hpp"

class DController {
public:
    DController(float kD, float alpha, float dt) : m_filter{ alpha }, m_k{ kD / dt } {}

    float update(float, float measurement) {
        float const filteredMeasurement = m_filter.update(measurement);

        float const output = -m_k * (filteredMeasurement - m_prevMeasurement);
        m_prevMeasurement = filteredMeasurement;
        return output;
    }

private:
    LagFilter m_filter;
    float const m_k{};

    float m_prevMeasurement = 0.0f;
};
