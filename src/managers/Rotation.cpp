#include "managers/Rotation.hpp"

#include "Constants.hpp"

#include "state/Radians.hpp"
#include "state/Vector.hpp"

#include <algorithm>

float Rotation::update(Radians currentAngle) {
    Radians const angularError = m_targetAngle - currentAngle;

    float const angularVelocity = m_rotationController.update(angularError, 0.0f);
    return std::clamp(angularVelocity, -Manager::Rotation::MAX_SPEED, Manager::Rotation::MAX_SPEED);
}
