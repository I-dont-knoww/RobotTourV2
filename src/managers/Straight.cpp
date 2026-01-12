#include "managers/Straight.hpp"

#include "Constants.hpp"

#include "state/Vector.hpp"

#include <algorithm>
#include <cmath>
#include <optional>

Straight::Straight(float dt)
    : m_headingController{ { Manager::Straight::angularKp },
                           { Manager::Straight::angularKd, Manager::Straight::FILTER_ALPHA, dt } },
      m_linearController{ { Manager::Straight::linearKp },
                          { Manager::Straight::linearKd, Manager::Straight::FILTER_ALPHA, dt } },
      m_centripetalFilter{ Manager::Straight::CENTRIPETAL_FILTER_CUTOFF, dt } {}

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

float getLinearControl(float headingError, float turnAngle) {
    using Manager::Straight::LINEAR_CONTROL_AUTHORITY;

    float const angleBound = std::max(Constants::PI / 16.0f, std::fabsf(turnAngle / 2.0f));
    if (headingError < -angleBound || headingError > angleBound) return 0.0f;
    else return LINEAR_CONTROL_AUTHORITY;
}

float getAngularSpeed(float headingErrorSpeed, float linearErrorSpeed, float linearAuthority) {
    float const headingErrorSpeedMagnitude = std::fabsf(headingErrorSpeed) + linearAuthority;
    return std::clamp(linearErrorSpeed, -headingErrorSpeedMagnitude, headingErrorSpeedMagnitude) +
           headingErrorSpeed;
}

float getDistanceLeft(Vec2 const& targetPosition, Vec2 const& currentPosition) {
    return (targetPosition - currentPosition).length();
}

float getSlowdownSpeed(std::optional<float> finalSpeed, float distanceLeft, float stoppingRadius) {
    using Manager::Straight::MAX_LINEAR_SPEED;
    using Manager::Straight::slowdownKh;
    using Manager::Straight::slowdownKp;
    using Manager::Straight::slowdownKs;

    if (!finalSpeed.has_value()) return MAX_LINEAR_SPEED;

    return slowdownKp * std::sqrtf(slowdownKh * (distanceLeft - stoppingRadius)) + slowdownKs +
           *finalSpeed;
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

Vec2 Straight::limitSpeeds(float linearSpeed, float angularSpeed) {
    using Manager::Straight::MAX_CENTRIPETAL;
    using Manager::Straight::MAX_LINEAR_SPEED;
    using Manager::Straight::TURN_ANGULAR_SPEED;

    angularSpeed = std::clamp(angularSpeed, -TURN_ANGULAR_SPEED, TURN_ANGULAR_SPEED);

    if (angularSpeed != 0.0f) {
        float const maxLinearSpeed = MAX_CENTRIPETAL / std::fabsf(angularSpeed);
        float const filteredMaxLinearSpeed = std::min(MAX_LINEAR_SPEED,
                                                      m_centripetalFilter.update(maxLinearSpeed));

        linearSpeed = std::clamp(linearSpeed, -filteredMaxLinearSpeed, filteredMaxLinearSpeed);
    }

    return { linearSpeed, angularSpeed };
}

void Straight::set(Vec2 const& startPosition, Vec2 const& targetPosition, float targetTime,
                   float stoppingRadius, float turnAngle, bool reverse, bool stop) {
    using Chassis::MASS;
    using Manager::Follower::TURNING_RADIUS;
    using Manager::Straight::MAX_CENTRIPETAL;

    m_startPosition = startPosition;
    m_targetPosition = targetPosition;
    m_targetAngle = (targetPosition - startPosition).angle();
    m_targetTime = targetTime;
    m_turnAngle = turnAngle;
    m_stoppingRadius = stoppingRadius;

    if (stop) m_finalSpeed = 0.0f;
    else if (turnAngle == 0.0f) m_finalSpeed = std::nullopt;
    else {
        float const turnRadius = TURNING_RADIUS *
                                 std::fabsf(std::tanf((Constants::PI - turnAngle) / 2.0f));
        m_finalSpeed = std::sqrtf(MAX_CENTRIPETAL * turnRadius / MASS);
    }

    m_reverse = reverse;
}

Vec2 Straight::update(Vec2 const& currentPosition, Radians currentAngle, 
                      float currentTime) {
    float const headingError = getHeadingError(m_targetAngle, currentAngle, m_reverse);
    float const headingErrorSpeed = m_headingController.update(0.0f, headingError);
    float const linearError = getLinearError(m_startPosition, m_targetPosition, currentPosition);
    float const linearErrorSpeed = m_linearController.update(0.0f, linearError);

    float const linearControl = getLinearControl(headingError, m_turnAngle);
    float const angularSpeed = getAngularSpeed(headingErrorSpeed, linearErrorSpeed, linearControl);

    float const distanceLeft = getDistanceLeft(m_targetPosition, currentPosition);
    float const slowdownSpeed = getSlowdownSpeed(m_finalSpeed, distanceLeft, m_stoppingRadius);
    auto const targetSpeed = getTargetSpeed(distanceLeft, m_targetTime, currentTime);
    float const linearSpeed = getLinearSpeed(targetSpeed, slowdownSpeed, m_reverse);

    return limitSpeeds(linearSpeed, angularSpeed);
}
