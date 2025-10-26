#pragma once

#include "Constants.hpp"

#include "control/Controller.hpp"
#include "control/pid/DController.hpp"
#include "control/pid/PController.hpp"

#include "state/Radians.hpp"
#include "state/Vector.hpp"

class Heading {
public:
    Heading() = default;

    void set(Vec2 const& startPosition, Vec2 const& targetPosition, bool reverse) {
        m_startPosition = startPosition;
        m_targetPosition = targetPosition;

        Radians const angleToDestination = (targetPosition - startPosition).angle();
        m_targetAngle = reverse ? angleToDestination + Radians{ Constants::PI }
                                : angleToDestination;
    }

    // float update(Vec2 const& currentPosition, Radians currentAngle, float dt);
    float update(Vec2 const& currentPosition, Vec2 const& currentVelocity, Radians currentAngle,
                 float dt);

private:
    Controller<PController, DController> m_angularController{ { Manager::Heading::angularKp },
                                                              { Manager::Heading::angularKd,
                                                                Manager::Heading::angularAlpha } };

    Vec2 m_startPosition{};
    Vec2 m_targetPosition{};
    Radians m_targetAngle{};
};
