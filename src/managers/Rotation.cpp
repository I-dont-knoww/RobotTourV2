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

static float getMaxSpeed(float currentTime, float startTime) {
    using Manager::Rotation::MAX_SPEED;
    using Manager::Rotation::SPEEDUP_ACCEL;

    float const maxAccelerationSpeed = SPEEDUP_ACCEL * (currentTime - startTime);
    return std::min(maxAccelerationSpeed, MAX_SPEED);
}

Vec2 Rotation::update(Radians currentAngle, float currentTime) {
    using Manager::Rotation::GRABBING_SPEED;

    Radians const angularError = m_targetAngle - currentAngle;

    float const targetSpeed = getTargetSpeed(angularError);
    float const maxSpeed = getMaxSpeed(currentTime, m_startTime);
    float const clampedSpeed = std::clamp(targetSpeed, -maxSpeed, maxSpeed);

    return Vec2{ GRABBING_SPEED, clampedSpeed };
}
