#pragma once

#include "Constants.hpp"

#include "control/Controller.hpp"
#include "control/feedforward/SController.hpp"
#include "control/pid/DController.hpp"
#include "control/pid/PController.hpp"

#include "filters/RCFilter.hpp"

#include "path/Path.hpp"

#include "state/Radians.hpp"
#include "state/Vector.hpp"

#include <optional>

class Straight {
public:
    struct Movement {
        Path path{};
        float targetTime{};
    };

    Straight(float dt);

    void set(Vec2 const& startPosition, Movement const& currentMovement,
             std::optional<Movement> const& nextMovement, float stoppingRadius);

    Vec2 update(Vec2 const& currentPosition, Radians currentAngle, float currentTime);

private:
    float getLinearSpeed(std::optional<float> targetSpeed, float slowdownSpeed, bool reverse);

    Vec2 limitSpeeds(float linearSpeed, float angularSpeed);

    Controller<PController, DController> m_headingController;
    Controller<PController, DController> m_linearController;

    RCFilter m_angularSpeedFilter;

    Vec2 m_startPosition{};
    Vec2 m_targetPosition{};

    std::optional<float> m_finalSpeed{};

    Radians m_targetAngle{};
    float m_targetTime{};

    float m_turnAngle{};
    float m_stoppingRadius{};

    bool m_reverse{};
};
