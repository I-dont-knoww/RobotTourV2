#pragma once

#include "Constants.hpp"

#include "control/Controller.hpp"
#include "control/feedforward/SController.hpp"
#include "control/pid/PController.hpp"

#include "state/Radians.hpp"
#include "state/Vector.hpp"

#include <optional>

class Rotation {
public:
    Rotation() = default;

    Radians targetAngle() const { return m_targetAngle; }

    void set(Radians targetAngle) { m_targetAngle = targetAngle; }

    std::optional<Vec2> update(Radians currentAngle);

private:
    Controller<SController, PController> m_rotationController{ { Manager::Rotation::kS },
                                                               { Manager::Rotation::kP } };

    Radians m_targetAngle{};
};
