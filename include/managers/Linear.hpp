#pragma once

#include "Constants.hpp"

#include "control/Controller.hpp"
#include "control/feedforward/SController.hpp"
#include "control/pid/PController.hpp"

#include "state/Vector.hpp"

class Linear {
public:
    Linear() = default;

    void set(Vec2 const& startPosition, Vec2 const& targetPosition, float targetTime, bool reverse,
             bool stop) {
        m_startPosition = startPosition;
        m_targetPosition = targetPosition;

        m_targetTime = targetTime;

        m_reverse = reverse;
        m_stop = stop;
    }

    Vec2 targetPosition() const { return m_targetPosition; };

    float update(Vec2 const& currentPosition, Vec2 const& currentVelocity, float angularVelocity,
                 float currentTime, float dt);

private:
    Controller<PController, SController> m_positionController{ { Manager::Position::kP },
                                                               { Manager::Position::kS } };

    Vec2 m_startPosition{};
    Vec2 m_targetPosition{};

    float m_targetTime{};

    bool m_reverse{};
    bool m_stop{};
};
