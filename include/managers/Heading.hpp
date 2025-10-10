#pragma once

#include "Constants.hpp"

#include "control/Controller.hpp"
#include "control/pid/PController.hpp"

#include "state/Radians.hpp"
#include "state/Vector.hpp"

class Heading {
public:
    Heading() = default;

    void set(Vec2 const& targetPosition, bool reverse) {
        m_targetPosition = targetPosition;
        m_reverse = reverse;
    }

    float update(Vec2 const& currentPosition, float dt);

private:
    Vec2 m_targetPosition{};
    bool m_reverse{};
};
