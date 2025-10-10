#pragma once

#include "Constants.hpp"

class Fusion {
public:
    Fusion() = default;

    float update(float angularVelocity, float dt) {
        m_heading += angularVelocity * dt;
        return m_heading;
    }

private:
    float m_heading = Constants::PI / 2.0f;
};
