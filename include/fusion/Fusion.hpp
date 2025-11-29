#pragma once

#include "Constants.hpp"

class Fusion {
public:
    Fusion(float dt);

    float update(float angularVelocity);

private:
    float m_heading = Constants::PI / 2.0f;
    float const m_dt{};
};
