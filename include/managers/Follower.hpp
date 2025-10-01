#pragma once

#include "Constants.hpp"

#include "managers/ExitCondition.hpp"
#include "managers/Heading.hpp"
#include "managers/Linear.hpp"
#include "managers/Rotation.hpp"

#include "path/Path.hpp"

#include "state/Radians.hpp"
#include "state/Vector.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>

using uint = unsigned int;

template <size_t N>
class Follower {
public:
    Follower(std::array<Path, N> path, std::array<float, N> targetTimes)
        : m_path{ path }, m_targetTimes{ targetTimes } {
        setupNextMode({ 0.0f, 0.0f });
    }

    bool finished() { return m_finished; }

    Vec2 update(Vec2 const& currentPosition, Radians currentAngle, float currentTime, float dt) {
        if (m_finished) return { 0.0f, 0.0f };

        float linearSpeed = 0.0f;
        float angularSpeed = 0.0f;

        if (m_mode == movement) {
            linearSpeed = m_linearManager.update(currentPosition, currentTime, dt);
            angularSpeed = m_headingManager.update(currentPosition, currentAngle, dt);
        } else if (m_mode == rotation) angularSpeed = m_rotationManager.update(currentAngle, dt);

        if (m_exitCondition.check(m_linearManager.targetPosition(), currentPosition,
                                  m_rotationManager.targetAngle(), currentAngle))
            m_finished = setupNextMode(currentPosition);

        // std::printf(">linearSpeed:%.5f\n>angularSpeed:%.5f\n>mode:%d\n>index:%d\n", linearSpeed,
        //             angularSpeed, m_mode, m_index);
        return { linearSpeed, angularSpeed };
    }

private:
    void setupMovement(Vec2 const& currentPosition) {
        Path const& path = m_path[m_index];
        uint const flags = path.flags;
        float const targetTime = m_targetTimes[m_index];

        m_linearManager.set(currentPosition, path.position, targetTime, flags & Path::REVERSE,
                            flags & Path::STOP);
        m_headingManager.set(path.position, flags & Path::REVERSE);

        float distanceThreshold = Manager::Follower::TURNING_RADIUS;

        if (flags & Path::ACCURATE)
            distanceThreshold = Manager::Follower::DISTANCE_THRESHOLD_ACCURATE;
        else if (flags & Path::STOP) distanceThreshold = Manager::Follower::DISTANCE_THRESHOLD_FAST;

        m_exitCondition.set(distanceThreshold, ExitCondition::NO_THRESHOLD);

        m_mode = movement;
    }

    void setupRotation() {
        Path const& previousPath = m_path[m_index - 1];
        Path const& path = m_path[m_index];

        m_rotationManager.set((path.position - previousPath.position).angle());
        m_exitCondition.set(ExitCondition::NO_THRESHOLD, Manager::Follower::ANGLE_THRESHOLD);

        m_mode = rotation;
    }

    bool setupNextMode(Vec2 const& currentPosition) {
        if (m_index == N) return true;

        if (m_mode == none || m_mode == rotation) setupMovement(currentPosition);
        else if (m_mode == movement) {
            ++m_index;
            if (m_index == N) return true;

            if (m_path[m_index - 1].flags & Path::STOP) setupRotation();
            else setupMovement(currentPosition);
        }

        return false;
    }

    enum Mode { none, movement, rotation };

    std::array<Path, N> m_path{};
    std::array<float, N> m_targetTimes{};

    Movement m_linearManager{};
    Heading m_headingManager{};
    Rotation m_rotationManager{};

    ExitCondition m_exitCondition{};

    Vec2 m_targetPosition{ 0.0f, 0.0f };
    Radians m_targetAngle{ 0.0f };

    Mode m_mode{ none };
    size_t m_index = 0u;

    bool m_finished{ false };
};
