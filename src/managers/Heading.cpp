#include "managers/Heading.hpp"

#include "Constants.hpp"

#include "state/Radians.hpp"
#include "state/Vector.hpp"

#include <algorithm>

float Heading::update(Vec2 const& currentPosition, Radians currentAngle, float dt) {
    float reverse = m_reverse ? -1.0f : 1.0f;

    Radians const targetAngle = (m_targetPosition - currentPosition).angle();
    Radians const error = targetAngle - currentAngle;

    float const angularVelocity = -reverse * m_headingController.update(0.0f, error, dt);

    // std::printf(">angularOutput:%.5f\n>angularError:%.5f\n", angularVelocity, error.toFloat());
    // std::printf(">currentAngle:%.5f\n>targetAngle:%.5f\n", currentAngle.toFloat(),
    //             targetAngle.toFloat());

    return std::clamp(angularVelocity, -Constants::PI, Constants::PI);
}
