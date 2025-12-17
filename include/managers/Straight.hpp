#pragma once

#include "Constants.hpp"

#include "control/Controller.hpp"
#include "control/feedforward/SController.hpp"
#include "control/pid/DController.hpp"
#include "control/pid/PController.hpp"

#include "filters/RCFilter.hpp"

#include "state/Radians.hpp"
#include "state/Vector.hpp"

#include <optional>

class Straight {
public:
    Straight(float dt);

    void set(Vec2 const& startPosition, Vec2 const& targetPosition, float targetTime,
             float stoppingRadius, float turnAngle, bool reverse, bool stop);

    Vec2 update(Vec2 const& currentPosition, Radians currentAngle, float angularVelocity,
                float currentTime);

private:
    Vec2 limitSpeeds(float linearSpeed, float angularSpeed);

    Controller<PController, DController> m_headingController;
    Controller<PController, DController> m_linearController;

    RCFilter m_centripetalFilter;

    Vec2 m_startPosition{};
    Vec2 m_targetPosition{};

    std::optional<float> m_finalSpeed{};

    Radians m_targetAngle{};
    float m_targetTime{};

    float m_stoppingRadius{};

    bool m_reverse{};
};
