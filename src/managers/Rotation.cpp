#include "managers/Rotation.hpp"

#include "Constants.hpp"

#include "state/Radians.hpp"
#include "state/Vector.hpp"

#include <algorithm>

float Rotation::update(Radians currentAngle, float dt) {
    Radians const error = m_targetAngle - currentAngle;

    float const angularVelocity = m_rotationController.update(error, 0.0f, dt);
    return std::clamp(angularVelocity, -Constants::PI, Constants::PI);
}
