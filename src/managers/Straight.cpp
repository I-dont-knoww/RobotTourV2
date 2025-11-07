#include "managers/Straight.hpp"

#include "Constants.hpp"

#include "state/Vector.hpp"

#include <algorithm>
#include <cmath>
#include <optional>

#include <cstdio>

float getHeadingError(Radians targetAngle, Radians currentAngle, bool reverse) {
    Radians headingError = currentAngle - targetAngle;
    if (reverse) return headingError + Radians{ Constants::PI };
    else return headingError;
}

float getLinearError(Vec2 const& startPosition, Vec2 const& targetPosition,
                     Vec2 const& currentPosition) {
    Vec2 const pathVector = targetPosition - startPosition;
    Vec2 const startToCurrentPosition = currentPosition - startPosition;
    float const lateralError = Vec2::cross(pathVector, startToCurrentPosition) /
                               pathVector.length();

    return lateralError;
}

float getAngularSpeed(float headingErrorSpeed, float linearErrorSpeed, bool reverse) {
    return linearErrorSpeed + headingErrorSpeed;
}

float getDistanceLeft(Vec2 const& targetPosition, Vec2 const& currentPosition) {
    return (targetPosition - currentPosition).length();
}

float getSlowdownSpeed(float distanceLeft, Vec2 const& currentVelocity, float finalSpeed) {
    using Manager::Straight::slowdownKp;
    using Manager::Straight::slowdownKs;

    float const currentSpeed = currentVelocity.length();
    float const targetSpeed = slowdownKp * (distanceLeft + finalSpeed) + slowdownKs;

    return targetSpeed;
}

std::optional<float> getTargetSpeed(float distanceLeft, float targetTime, float currentTime) {
    if (targetTime <= currentTime) return std::nullopt;
    return distanceLeft / (targetTime - currentTime);
}

float getLinearSpeed(std::optional<float> targetSpeed, float slowdownSpeed, bool reverse) {
    float linearSpeed;
    if (!targetSpeed.has_value()) linearSpeed = slowdownSpeed;
    else linearSpeed = std::min(*targetSpeed, slowdownSpeed);

    return (reverse ? -1.0f : 1.0f) * linearSpeed;
}

Vec2 limitSpeeds(float linearSpeed, float angularSpeed) {
    using Manager::Follower::TURNING_RADIUS;
    using Manager::Straight::TURN_ANGULAR_SPEED;
    using Manager::Straight::TURN_LINEAR_FACTOR;

    angularSpeed = std::clamp(angularSpeed, -TURN_ANGULAR_SPEED, TURN_ANGULAR_SPEED);

    if (angularSpeed != 0.0f) {
        float const MAX_LINEAR_SPEED = TURN_LINEAR_FACTOR *
                                       std::fabsf(TURNING_RADIUS / angularSpeed);
        linearSpeed = std::clamp(linearSpeed, -MAX_LINEAR_SPEED, MAX_LINEAR_SPEED);
    }

    return { linearSpeed, angularSpeed };
}

Vec2 Straight::update(Vec2 const& currentPosition, Vec2 const& currentVelocity,
                      Radians currentAngle, float angularVelocity, float currentTime, float dt) {
    float const headingError = getHeadingError(m_targetAngle, currentAngle, m_reverse);
    float const linearError = getLinearError(m_startPosition, m_targetPosition, currentPosition);
    float const headingErrorSpeed = m_headingController.update(0.0f, headingError, dt);
    float const linearErrorSpeed = m_linearController.update(0.0f, linearError, dt);
    float const angularSpeed = getAngularSpeed(headingErrorSpeed, linearErrorSpeed, m_reverse);

    float const distanceLeft = getDistanceLeft(m_targetPosition, currentPosition);
    float const slowdownSpeed = getSlowdownSpeed(distanceLeft, currentVelocity, m_finalSpeed);
    auto const targetSpeed = getTargetSpeed(distanceLeft, m_targetTime, currentTime);
    float const linearSpeed = getLinearSpeed(targetSpeed, slowdownSpeed, m_reverse);

    return limitSpeeds(linearSpeed, angularSpeed);
}
