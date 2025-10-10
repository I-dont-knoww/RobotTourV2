#pragma once

#include "state/Radians.hpp"
#include "state/Vector.hpp"

#include <limits>
#include <optional>

class ExitCondition {
public:
    struct PositionTarget {
        Vec2 startPosition{};
        Vec2 targetPosition{};
        float distanceThreshold{};
    };
    struct AngleTarget {
        Radians targetAngle{};
        float angleThreshold{};
    };

    ExitCondition() = default;

    void set(std::optional<PositionTarget> const& positionTarget,
             std::optional<AngleTarget> const& angleTarget) {
        m_positionTarget = positionTarget;
        m_angleTarget = angleTarget;
    }

    bool check(Vec2 const& currentPosition, Radians currentAngle) const;

private:
    std::optional<PositionTarget> m_positionTarget{};
    std::optional<AngleTarget> m_angleTarget{};
};
