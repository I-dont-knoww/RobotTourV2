#include "managers/Rotation.hpp"

#include "Constants.hpp"

#include "state/Radians.hpp"
#include "state/Vector.hpp"

#include <algorithm>
#include <cmath>
#include <optional>

Rotation::Rotation(float dt) : m_dt{ dt } {}

static float getTargetSpeed(float angularError) {
    using Manager::Rotation::SLOWDOWN_ACCEL;
    using Manager::Rotation::MIN_SPEED;

    if (angularError >= 0.0f)
        return std::sqrtf(MIN_SPEED * MIN_SPEED + 2.0f * SLOWDOWN_ACCEL * angularError);
    else return -std::sqrtf(MIN_SPEED * MIN_SPEED - 2.0f * SLOWDOWN_ACCEL * angularError);
}

Vec2 Rotation::update(Radians currentAngle) {
    using Manager::Rotation::GRABBING_SPEED;
    using Manager::Rotation::MAX_SPEED;

    Radians const angularError = m_targetAngle - currentAngle;

    float const targetSpeed = getTargetSpeed(angularError);
    float const clampedSpeed = std::clamp(targetSpeed, -MAX_SPEED, MAX_SPEED);

    return Vec2{ GRABBING_SPEED, clampedSpeed };
}
