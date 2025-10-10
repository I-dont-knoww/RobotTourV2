#include "managers/ExitCondition.hpp"

#include "state/Radians.hpp"
#include "state/Vector.hpp"

#include <cmath>
#include <optional>

bool ExitCondition::check(Vec2 const& currentPosition, Radians currentAngle) const {
    auto positionMeetsTarget = [&](PositionTarget const& target) {
        Vec2 const path = target.targetPosition - target.startPosition;
        bool const isPastTarget = Vec2::dot(currentPosition - target.startPosition, path) >
                                  Vec2::dot(path, path);
        bool const isWithinThreshold = (target.targetPosition - currentPosition).lengthSquared() <
                                       target.distanceThreshold * target.distanceThreshold;

        return isPastTarget || isWithinThreshold;
    };
    auto angleMeetsTarget = [=](AngleTarget const& target) {
        float const angleError = target.targetAngle - currentAngle;
        return angleError >= -target.angleThreshold && angleError <= target.angleThreshold;
    };

    bool const positionCheck = m_positionTarget.transform(positionMeetsTarget).value_or(false);
    bool const angleCheck = m_angleTarget.transform(angleMeetsTarget).value_or(false);

    return positionCheck || angleCheck;
}
