#pragma once

#include "Constants.hpp"

#include "control/Controller.hpp"
#include "control/pid/IController.hpp"
#include "control/pid/PController.hpp"

#include "state/Radians.hpp"
#include "state/Vector.hpp"

class Rotation {
public:
    Rotation() = default;

    Radians targetAngle() const { return m_targetAngle; }

    void set(Radians targetAngle) { m_targetAngle = targetAngle; }

    float update(Radians currentAngle, float dt);

private:
    Radians m_targetAngle{};
};
