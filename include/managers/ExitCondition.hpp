#pragma once

#include "state/Radians.hpp"
#include "state/Vector.hpp"

#include <limits>

class ExitCondition {
public:
    ExitCondition() = default;

    void set(float distanceThreshold, float angleTreshold) {
        m_distanceThreshold = distanceThreshold;
        m_angleThreshold = angleTreshold;
    }

    bool check(Vec2 const& targetPosition, Vec2 const& currentPosition, Radians targetAngle,
               Radians currentAngle) const;

    static constexpr float NO_THRESHOLD = std::numeric_limits<float>::max();

private:
    float m_distanceThreshold{};
    float m_angleThreshold{};
};
