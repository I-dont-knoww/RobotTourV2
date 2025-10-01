#pragma once

#include "Constants.hpp"

#include "control/Controller.hpp"
#include "control/pid/IController.hpp"
#include "control/pid/PController.hpp"

#include "state/Vector.hpp"

class Movement {
public:
    Movement() = default;

    void set(Vec2 const& startPosition, Vec2 const& targetPosition, float targetTime, bool reverse,
             bool stop) {
        m_startPosition = startPosition;
        m_targetPosition = targetPosition;

        m_targetTime = targetTime;

        m_reverse = reverse;
        m_stop = stop;
    }

    Vec2 targetPosition() const { return m_targetPosition; };

    float update(Vec2 const& currentPosition, float currentTime, float dt);

private:
    Controller<PController, IController> m_positionController{
        { Manager::Position::kP },
        { Manager::Position::kI, -Manager::Position::INTEGRATOR_BOUND,
          Manager::Position::INTEGRATOR_BOUND }
    };

    Vec2 m_startPosition{};
    Vec2 m_targetPosition{};

    float m_targetTime{};

    bool m_reverse{};
    bool m_stop{};
};
