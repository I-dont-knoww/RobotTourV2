#pragma once

#include "Constants.hpp"

#include "kinematics/ForwardKinematics.hpp"

#include "managers/Rotation.hpp"
#include "managers/Straight.hpp"

#include "course/Course.hpp"

#include "state/Radians.hpp"
#include "state/Vector.hpp"

#include <cstddef>
#include <iterator>
#include <optional>

using uint = unsigned int;

template <size_t segmentCount, size_t routeCount>
class Follower {
public:
    Follower(Course::Course<segmentCount, routeCount> const& course, float dt) :
        m_course{ course }, m_straightManager{ dt }, m_rotationManager{} {
        setupNextMode();
    }

    bool finished() { return m_finished; }

    Vec2 update(ForwardKinematics::State const& state, float currentTime) {
        if (m_finished) return { 0.0f, 0.0f };

        std::optional<Vec2> const targetVelocities = updateCurrentManager(state, currentTime);
        if (!targetVelocities) {
            if (setupNextMode()) m_finished = true;
            return { 0.0f, 0.0f };
        } else return *targetVelocities;
    }

private:
    std::optional<Vec2> updateCurrentManager(ForwardKinematics::State const& state,
                                             float currentTime) {
        if (m_mode == movement)
            return m_straightManager.update(state.position, state.velocity, state.angle,
                                            currentTime);
        else if (m_mode == rotation) return m_rotationManager.update(state.angle);
        else return std::nullopt;
    }

    void setupMovement() {
        Course::Route const& route = m_course.routes[m_routeIndex];

        size_t const beginIndex = route.beginIndex;
        size_t const endIndex = route.endIndex;
        auto const beginIterator = std::begin(m_course.segments) + beginIndex;
        auto const endIterator = std::begin(m_course.segments) + endIndex;

        m_straightManager.setup({ beginIterator, endIterator }, route.targetTime);
        m_mode = movement;
    }

    void setupRotation() {
        Course::Route const& currentRoute = m_course.routes[m_routeIndex];

        Vec2 const& startPosition = m_course.segments[currentRoute.beginIndex].position;
        Vec2 const& nextPosition = m_course.segments[currentRoute.beginIndex + 1].position;
        Radians const targetAngle = (nextPosition - startPosition).angle();

        m_rotationManager.set(targetAngle);
        m_mode = rotation;
    }

    bool setupNextMode() {
        if (m_routeIndex == routeCount) return true;

        if (m_mode == none || m_mode == rotation) setupMovement();
        else if (m_mode == movement) {
            ++m_routeIndex;
            if (m_routeIndex == routeCount) return true;

            setupRotation();
        }

        return false;
    }

    enum Mode { none, movement, rotation };

    Course::Course<segmentCount, routeCount> const& m_course{};

    Straight m_straightManager;
    Rotation m_rotationManager;

    Mode m_mode = none;
    size_t m_routeIndex = 0;

    bool m_finished = false;
};
