#pragma once

#include "state/Vector.hpp"

class InverseKinematics {
public:
    InverseKinematics() = default;

    Vec2 update(float linearVelocity, float angularVelocity);
};
