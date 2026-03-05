#pragma once

#include "Constants.hpp"

#include "filters/RCFilter.hpp"

class Fusion {
public:
    Fusion(float dt);

    float update(float angularVelocity);

private:
    RCFilter m_gyroscopeFilter;

    float m_heading = Constants::PI / 2.0f;
    float const m_dt{};
};
