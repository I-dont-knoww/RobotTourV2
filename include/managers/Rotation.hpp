#pragma once

#include "Constants.hpp"

#include "state/Radians.hpp"
#include "state/Vector.hpp"

class Rotation {
public:
    Rotation(float dt);

    Radians targetAngle() const { return m_targetAngle; }

    void set(Radians targetAngle) { m_targetAngle = targetAngle; }

    Vec2 update(Radians currentAngle);

private:
    Radians m_targetAngle{};
    float m_dt{};
};
