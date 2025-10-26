#include "managers/Heading.hpp"

#include "Constants.hpp"

#include "state/Radians.hpp"
#include "state/Vector.hpp"

#include <algorithm>
#include <cmath>

#include <cstdio>

// float Heading::update(Vec2 const& currentPosition, Radians currentAngle, float dt) {
//     Radians const angularError = m_targetAngle - currentAngle;
//     float const targetHeadingAngularVelocity = m_headingController.update(angularError, 0.0f,
//     dt);

//     Vec2 const pathVector = m_targetPosition - m_startPosition;
//     Vec2 const startToCurrentPosition = currentPosition - m_startPosition;
//     float const lateralError = Vec2::cross(pathVector, startToCurrentPosition) /
//                                  pathVector.length();
//     float const targetLateralAngularVelocity = m_lateralController.update(0.0f, lateralError,
//     dt);

//     float const targetAngularVelocity = targetHeadingAngularVelocity +
//     targetLateralAngularVelocity;

//     return std::clamp(targetAngularVelocity, -Manager::Heading::MAX_SPEED,
//                       Manager::Heading::MAX_SPEED);
// }

float Heading::update(Vec2 const& currentPosition, Vec2 const& currentVelocity,
                      Radians currentAngle, float dt) {
    float const angularError = (m_targetAngle - currentAngle).toFloat();

    Vec2 const pathVector = m_targetPosition - m_startPosition;
    Vec2 const startToCurrentPosition = currentPosition - m_startPosition;
    float const lateralError = Vec2::cross(pathVector, startToCurrentPosition) /
                               pathVector.length();

    float const currentSpeed = currentVelocity.length();
    float targetAngularVelocity{};
    if (currentSpeed != 0.0f)
        targetAngularVelocity =
            m_angularController.update(angularError, 0.0f, dt) +
            Manager::Heading::lateralScale *
                -std::atanf(Manager::Heading::lateralK * lateralError / currentSpeed);
    else targetAngularVelocity = 0.0f;

    return std::clamp(targetAngularVelocity, -Manager::Heading::MAX_SPEED,
                      Manager::Heading::MAX_SPEED);
}
