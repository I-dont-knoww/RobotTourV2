#include "managers/ExitCondition.hpp"

#include "state/Radians.hpp"
#include "state/Vector.hpp"

#include <cmath>

bool ExitCondition::check(Vec2 const& targetPosition, Vec2 const& currentPosition,
                          Radians targetAngle, Radians currentAngle) const {
    float const positionDistance = (targetPosition - currentPosition).length();
    float const angleDistance = std::fabsf((targetAngle - currentAngle).toFloat());

    return positionDistance < m_distanceThreshold && angleDistance < m_angleThreshold;
}
