#include "managers/Rotation.hpp"

#include "Constants.hpp"

#include "state/Radians.hpp"
#include "state/Vector.hpp"

#include <algorithm>
#include <cmath>
#include <optional>

std::optional<Vec2> Rotation::update(Radians currentAngle) {
    using Manager::Follower::ANGLE_THRESHOLD;
    using Manager::Rotation::GRABBING_SPEED;
    using Manager::Rotation::MAX_SPEED;

    Radians const angularError = m_targetAngle - currentAngle;
    if (angularError >= -ANGLE_THRESHOLD && angularError <= ANGLE_THRESHOLD) return std::nullopt;

    float const angularVelocity = m_rotationController.update(angularError, 0.0f);
    float const clampedAngularVelocity = std::clamp(angularVelocity, -MAX_SPEED, MAX_SPEED);

    return Vec2{ GRABBING_SPEED, clampedAngularVelocity };
}
