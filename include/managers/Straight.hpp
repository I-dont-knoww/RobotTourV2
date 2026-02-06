#pragma once

#include "Constants.hpp"

#include "filters/RCFilter.hpp"

#include "course/Course.hpp"

#include "state/Radians.hpp"
#include "state/Vector.hpp"

#include <optional>
#include <span>

class Straight {
public:
    using Route = std::span<const Course::Segment>;
    using RouteIterator = Route::const_iterator;

    Straight(float dt);

    void setup(Route route, float targetTime);

    std::optional<Vec2> update(Vec2 const& currentPosition, Vec2 const& currentVelocity,
                               Radians currentAngle, float currentTime);

private:
    Vec2 getNextGoalPoint(Vec2 const& currentPosition, float lookAheadDistance);
    RouteIterator getNextUnpassedSegment(Vec2 const& currentPosition);
    float getLimitedLinearVelocity(Vec2 const& currentPosition,
                                   RouteIterator unpassedSegmentIterator, float timeLeft);

    Vec2 m_previousGoalPoint{};

    Route m_route{};
    RouteIterator m_previousGoalPointIterator{};
    RouteIterator m_previousPassedSegmentIterator{};

    float m_previousVelocity{};
    float m_targetTime{};

    float m_dt{};
};
