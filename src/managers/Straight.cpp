#include "managers/Straight.hpp"

#include "Constants.hpp"

#include "state/Vector.hpp"

#include <algorithm>
#include <cmath>
#include <optional>

Straight::Straight(float dt) :
    m_headingController{ { Manager::Straight::angularKp },
                         { Manager::Straight::angularKd, Manager::Straight::FILTER_ALPHA, dt } },
    m_linearController{ { Manager::Straight::linearKp },
                        { Manager::Straight::linearKd, Manager::Straight::FILTER_ALPHA, dt } },
    m_angularSpeedFilter{ Manager::Straight::CENTRIPETAL_FILTER_CUTOFF, dt } {}

static float getHeadingError(Radians targetAngle, Radians currentAngle, bool reverse) {
    Radians headingError = currentAngle - targetAngle;
    if (reverse) return headingError + Radians{ Constants::PI };
    else return headingError;
}

static float getLinearError(Vec2 const& startPosition, Vec2 const& targetPosition,
                            Vec2 const& currentPosition) {
    Vec2 const pathVector = targetPosition - startPosition;
    Vec2 const startToCurrentPosition = currentPosition - startPosition;
    float const lateralError = Vec2::cross(pathVector, startToCurrentPosition) /
                               pathVector.length();

    return lateralError;
}

static float getLinearControl(float headingError, float turnAngle) {
    using Manager::Straight::LINEAR_CONTROL_AUTHORITY;

    float const angleBound = std::max(Constants::PI / 16.0f, std::fabsf(turnAngle / 2.0f));
    if (headingError < -angleBound || headingError > angleBound) return 0.0f;
    else return LINEAR_CONTROL_AUTHORITY;
}

static float getAngularSpeed(float headingErrorSpeed, float linearErrorSpeed,
                             float linearAuthority) {
    float const headingErrorSpeedMagnitude = std::fabsf(headingErrorSpeed) + linearAuthority;
    return std::clamp(linearErrorSpeed, -headingErrorSpeedMagnitude, headingErrorSpeedMagnitude) +
           headingErrorSpeed;
}

static float getDistanceLeft(Vec2 const& targetPosition, Vec2 const& currentPosition) {
    return (targetPosition - currentPosition).length();
}

static float getSlowdownSpeed(std::optional<float> finalSpeed, float distanceLeft,
                              float stoppingRadius) {
    using Manager::Straight::MAX_LINEAR_SPEED;
    using Manager::Straight::SLOWDOWN_ACCEL;

    if (!finalSpeed) return MAX_LINEAR_SPEED;

    float const slowdownSpeedSquared = *finalSpeed * *finalSpeed +
                                       2.0f * SLOWDOWN_ACCEL * (distanceLeft - stoppingRadius);

    if (slowdownSpeedSquared <= 0.0f) return *finalSpeed;
    else return std::sqrtf(slowdownSpeedSquared);
}

static std::optional<float> getTargetSpeed(float targetTime, float currentTime, float distanceLeft,
                                           std::optional<float> finalSpeed) {
    using Manager::Straight::MAX_LINEAR_SPEED;
    using Manager::Straight::SLOWDOWN_ACCEL;

    if (!finalSpeed) return std::nullopt;

    float const timeLeft = targetTime - currentTime;
    if (timeLeft <= 0.0f) return std::nullopt;
    if (distanceLeft / timeLeft <= *finalSpeed) return distanceLeft / timeLeft;

    float const determinant = SLOWDOWN_ACCEL * SLOWDOWN_ACCEL * timeLeft * timeLeft +
                              2.0f * SLOWDOWN_ACCEL * (*finalSpeed * timeLeft - distanceLeft);
    if (determinant <= 0.0f) return std::nullopt;
    else return *finalSpeed + SLOWDOWN_ACCEL * timeLeft - std::sqrtf(determinant);
}

float Straight::getLinearSpeed(std::optional<float> targetSpeed, float slowdownSpeed,
                               bool reverse) {
    float linearSpeed;
    if (!targetSpeed) linearSpeed = slowdownSpeed;
    else linearSpeed = std::min(*targetSpeed, slowdownSpeed);

    return (reverse ? -1.0f : 1.0f) * linearSpeed;
}

Vec2 Straight::limitSpeeds(float linearSpeed, float angularSpeed) {
    using Chassis::MASS;
    using Manager::Straight::MAX_CENTRIPETAL;
    using Manager::Straight::MAX_LINEAR_SPEED;
    using Manager::Straight::TURN_ANGULAR_SPEED;

    angularSpeed = std::clamp(angularSpeed, -TURN_ANGULAR_SPEED, TURN_ANGULAR_SPEED);
    float const filteredAngularSpeed = m_angularSpeedFilter.update(angularSpeed);

    if (angularSpeed != 0.0f) {
        float const maxLinearSpeed = MAX_CENTRIPETAL / std::fabsf(filteredAngularSpeed) / MASS;
        linearSpeed = std::clamp(linearSpeed, -maxLinearSpeed, maxLinearSpeed);
    }

    return { linearSpeed, angularSpeed };
}

static float getTurnAngle(Vec2 const& startPosition, Straight::Movement const& currentMovement,
                          std::optional<Straight::Movement> const& nextMovement) {
    if (!nextMovement) return 0.0f;
    else {
        Vec2 const currentDirection = currentMovement.path.position - startPosition;
        Vec2 const nextDirection = nextMovement->path.position - currentMovement.path.position;

        return std::acosf(Vec2::dot(currentDirection, nextDirection) /
                          (currentDirection.length() * nextDirection.length()));
    }
}

static std::optional<float> getFinalSpeed(std::optional<Straight::Movement> const& nextMovement,
                                          Straight::Movement const& currentMovement,
                                          float stoppingRadius, float turnAngle) {
    using Chassis::MASS;
    using Manager::Follower::DISTANCE_THRESHOLD_ACCURATE;
    using Manager::Follower::TURNING_RADIUS;
    using Manager::Straight::MAX_CENTRIPETAL;
    using Manager::Straight::MAX_LINEAR_SPEED;
    using Manager::Straight::SLOWDOWN_MIN_SPEED;

    if (currentMovement.path.flags & Path::STOP || !nextMovement) return SLOWDOWN_MIN_SPEED;
    else if (turnAngle == 0.0f) return MAX_LINEAR_SPEED;
    else {
        float const turnRadius = TURNING_RADIUS *
                                 std::fabsf(std::tanf((Constants::PI - turnAngle) / 2.0f));
        float const nextTravelLength =
            (nextMovement->path.position - currentMovement.path.position).length();

        float const maxTurnSpeed = std::sqrtf(MAX_CENTRIPETAL * turnRadius / MASS);
        float const slowdownSpeed = nextMovement->path.flags & Path::STOP
                                        ? getSlowdownSpeed(0.0f, nextTravelLength,
                                                           DISTANCE_THRESHOLD_ACCURATE)
                                        : MAX_LINEAR_SPEED;
        float const nextTargetTime = nextMovement->targetTime - currentMovement.targetTime;
        float const nextSpeed = nextTargetTime != 0.0f ? nextTravelLength / nextTargetTime
                                                       : MAX_LINEAR_SPEED;

        return std::min({ maxTurnSpeed, slowdownSpeed, nextSpeed, MAX_LINEAR_SPEED });
    }
}

void Straight::set(Vec2 const& startPosition, Movement const& currentMovement,
                   std::optional<Movement> const& nextMovement, float stoppingRadius) {
    m_startPosition = startPosition;
    m_targetPosition = currentMovement.path.position;
    m_stoppingRadius = stoppingRadius;

    m_targetAngle = (m_targetPosition - m_startPosition).angle();
    m_turnAngle = getTurnAngle(startPosition, currentMovement, nextMovement);
    m_targetTime = currentMovement.targetTime;

    m_finalSpeed = getFinalSpeed(nextMovement, currentMovement, m_stoppingRadius, m_turnAngle);

    m_reverse = currentMovement.path.flags & Path::REVERSE;
}

Vec2 Straight::update(Vec2 const& currentPosition, Radians currentAngle, float currentTime) {
    float const headingError = getHeadingError(m_targetAngle, currentAngle, m_reverse);
    float const headingErrorSpeed = m_headingController.update(0.0f, headingError);
    float const linearError = getLinearError(m_startPosition, m_targetPosition, currentPosition);
    float const linearErrorSpeed = m_linearController.update(0.0f, linearError);

    float const linearControl = getLinearControl(headingError, m_turnAngle);
    float const angularSpeed = getAngularSpeed(headingErrorSpeed, linearErrorSpeed, linearControl);

    float const distanceLeft = getDistanceLeft(m_targetPosition, currentPosition);
    float const slowdownSpeed = getSlowdownSpeed(m_finalSpeed, distanceLeft, m_stoppingRadius);
    auto const targetSpeed = getTargetSpeed(m_targetTime, currentTime, distanceLeft, m_finalSpeed);
    float const linearSpeed = getLinearSpeed(targetSpeed, slowdownSpeed, m_reverse);

    return limitSpeeds(linearSpeed, angularSpeed);
}
