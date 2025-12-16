#pragma once

#include "Constants.hpp"

#include "kinematics/ForwardKinematics.hpp"

#include "managers/ExitCondition.hpp"
#include "managers/Rotation.hpp"
#include "managers/Straight.hpp"

#include "path/Path.hpp"

#include "state/Radians.hpp"
#include "state/Vector.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <optional>

using uint = unsigned int;

template <size_t N>
class Follower {
public:
    Follower(std::array<Path, N> const& path, std::array<float, N> const& targetTimes, float dt)
        : m_path{ path },
          m_targetTimes{ targetTimes },
          m_straightManager{ dt },
          m_rotationManager{} {
        setupNextMode({ 0.0f, 0.0f });
    }

    bool finished() { return m_finished; }

    Vec2 update(ForwardKinematics::State const& state, float currentTime) {
        if (m_exitCondition.check(state.position, state.angle))
            m_finished = setupNextMode(state.position);
        if (m_finished) return { 0.0f, 0.0f };

        if (m_mode == movement)
            return m_straightManager.update(state.position, state.angle, state.angularVelocity,
                                            currentTime);
        else if (m_mode == rotation) return m_rotationManager.update(state.angle);
        else return { 0.0f, 0.0f };
    }

private:
    void setupMovement(Vec2 const& currentPosition) {
        Path const& path = m_path[m_index];
        uint const flags = path.flags;
        float const targetTime = m_targetTimes[m_index];

        Vec2 const& previousPathPosition = (m_index == 0u) ? Vec2{ 0.0f, 0.0f }
                                                           : m_path[m_index - 1u].position;
        float turnAngle{};
        if (m_index == m_path.size() - 1u) turnAngle = 0.0f;
        else {
            Vec2 const& nextPathPosition = m_path[m_index + 1u].position;

            Vec2 const currentDirection = path.position - previousPathPosition;
            Vec2 const previousDirection = nextPathPosition - path.position;

            turnAngle = std::acosf(Vec2::dot(currentDirection, previousDirection) /
                                   (currentDirection.length() * previousDirection.length()));
        }

        float distanceThreshold = Manager::Follower::TURNING_RADIUS;
        if (flags & Path::ACCURATE)
            distanceThreshold = Manager::Follower::DISTANCE_THRESHOLD_ACCURATE;
        else if (flags & Path::STOP) distanceThreshold = Manager::Follower::DISTANCE_THRESHOLD_FAST;

        m_straightManager.set(previousPathPosition, path.position, targetTime, distanceThreshold,
                              turnAngle, flags & Path::REVERSE, flags & Path::STOP);
        m_exitCondition.set({ { currentPosition, path.position, distanceThreshold } },
                            std::nullopt);

        m_mode = movement;
    }

    void setupRotation() {
        Path const& previousPath = m_path[m_index - 1];
        Path const& path = m_path[m_index];

        Radians targetAngle{ (path.position - previousPath.position).angle() };
        if (path.flags & Path::REVERSE) targetAngle += Radians{ Constants::PI };

        m_rotationManager.set(targetAngle);
        m_exitCondition.set(std::nullopt, { { targetAngle, Manager::Follower::ANGLE_THRESHOLD } });

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

    enum Mode { none, wait, movement, rotation };

    std::array<Path, N> m_path{};
    std::array<float, N> m_targetTimes{};

    Straight m_straightManager;
    Rotation m_rotationManager;

    ExitCondition m_exitCondition{};

    Vec2 m_targetPosition{ 0.0f, 0.0f };
    Radians m_targetAngle{ 0.0f };

    Mode m_mode{ none };
    size_t m_index = 0u;

    bool m_finished{ false };
};
