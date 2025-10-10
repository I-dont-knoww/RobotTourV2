#pragma once

#include "Constants.hpp"

#include "control/Controller.hpp"
#include "control/feedfoward/AController.hpp"
#include "control/feedfoward/SController.hpp"
#include "control/feedfoward/VController.hpp"
#include "control/pid/DController.hpp"
#include "control/pid/IController.hpp"
#include "control/pid/PController.hpp"

#include "filters/LagFilter.hpp"
#include "filters/RCFilter.hpp"

#include "state/Radians.hpp"
#include "state/Vector.hpp"

class VelocityRegulator {
public:
    VelocityRegulator() = default;

    void setTargets(float targetLinearVelocity, Radians targetAngle) {
        m_targetLinearVelocity = targetLinearVelocity;
        m_targetAngle = targetAngle;
    }

    Vec2 update(Vec2 const& currentVelocity, Radians currentAngle, float batteryVoltage, float dt);

private:
    Controller<SController, VController, AController, PController, IController>
        m_linearVelocityController{
            { Regulators::Velocity::Linear::kS },
            { Regulators::Velocity::Linear::kV },
            { Regulators::Velocity::Linear::kA, Regulators::Velocity::Linear::CUTOFF_FREQUENCY },
            { Regulators::Velocity::Linear::kP },
            { Regulators::Velocity::Linear::kI, Regulators::Velocity::Linear::MIN_INT,
              Regulators::Velocity::Linear::MAX_INT }
        };
    Controller<SController, PController, IController> m_angleController{
        { Regulators::Velocity::Angular::kS },
        { Regulators::Velocity::Angular::kP },
        { Regulators::Velocity::Angular::kI, -Regulators::Velocity::Angular::INT_BOUND,
          Regulators::Velocity::Angular::INT_BOUND }
    };

    LagFilter m_linearVelocitySetpointFilter{ Regulators::Velocity::Linear::LAG_FILTER_K };

    float m_targetLinearVelocity{};
    Radians m_targetAngle{};
};
