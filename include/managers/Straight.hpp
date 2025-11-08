#pragma once

#include "Constants.hpp"

#include "control/Controller.hpp"
#include "control/feedforward/SController.hpp"
#include "control/pid/DController.hpp"
#include "control/pid/PController.hpp"

#include "state/Radians.hpp"
#include "state/Vector.hpp"

#include <optional>

class Straight {
public:
    Straight() = default;

    void set(Vec2 const& startPosition, Vec2 const& targetPosition, float targetTime,
             float turnAngle, bool reverse, bool stop);

    Vec2 update(Vec2 const& currentPosition, Radians currentAngle, float angularVelocity,
                float currentTime, float dt);

private:
    Controller<PController, DController> m_headingController{ { Manager::Straight::angularKp },
                                                              { Manager::Straight::angularKd,
                                                                Manager::Straight::FILTER_ALPHA } };
    Controller<PController, DController> m_linearController{ { Manager::Straight::linearKp },
                                                             { Manager::Straight::linearKd,
                                                               Manager::Straight::FILTER_ALPHA } };

    Vec2 m_startPosition{};
    Vec2 m_targetPosition{};

    std::optional<float> m_finalSpeed{};

    Radians m_targetAngle{};
    float m_targetTime{};

    bool m_reverse{};
};
