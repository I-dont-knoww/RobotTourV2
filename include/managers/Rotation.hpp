#pragma once

#include "Constants.hpp"

#include "state/Radians.hpp"
#include "state/Vector.hpp"

class Rotation {
public:
    Rotation(float dt);

    Radians targetAngle() const { return m_targetAngle; }

    void set(Radians targetAngle, float startTime) {
        m_targetAngle = targetAngle;
        m_startTime = startTime;
    }

    Vec2 update(Radians currentAngle, float currentTime);

private:
    Radians m_targetAngle{};
    float m_startTime{};

    float m_dt{};
};
