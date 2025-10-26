#pragma once

#include "Constants.hpp"

#include "control/Controller.hpp"
#include "control/feedforward/AController.hpp"
#include "control/feedforward/SController.hpp"
#include "control/feedforward/VController.hpp"
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

    void setTargets(float targetLinearVelocity, float targetAngularVelocity) {
        m_targetLinearVelocity = targetLinearVelocity;
        m_targetAngularVelocity = targetAngularVelocity;
    }

    Vec2 update(Vec2 const& currentVelocity, Radians currentAngle, float currentAngularVelocity, float batteryVoltage, float dt);

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
    Controller<SController, VController, AController, PController, IController>
        m_anglularVelocityController{
            { Regulators::Velocity::Angular::kS },
            { Regulators::Velocity::Angular::kV },
            { Regulators::Velocity::Angular::kA, Regulators::Velocity::Angular::CUTOFF_FREQUENCY },
            { Regulators::Velocity::Angular::kP },
            { Regulators::Velocity::Angular::kI, Regulators::Velocity::Angular::MIN_INT,
              Regulators::Velocity::Angular::MAX_INT }
        };

    LagFilter m_linearVelocitySetpointFilter{ Regulators::Velocity::Linear::LAG_FILTER_K };
    LagFilter m_angularVelocitySetpointFilter{ Regulators::Velocity::Angular::LAG_FILTER_K };

    float m_targetLinearVelocity{};
    float m_targetAngularVelocity{};
};
