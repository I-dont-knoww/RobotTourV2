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
    VelocityRegulator(float dt);

    void setTargets(float targetLinearVelocity, float targetAngularVelocity) {
        m_targetLinearVelocity = targetLinearVelocity;
        m_targetAngularVelocity = targetAngularVelocity;
    }

    Vec2 update(Vec2 const& currentVelocity, Radians currentAngle, float currentAngularVelocity,
                float batteryVoltage);

private:
    Controller<SController, VController, AController, PController, IController>
        m_linearVelocityController;
    Controller<SController, VController, AController, PController, IController>
        m_anglularVelocityController;

    RCFilter m_linearVelocityOutputFilter;

    LagFilter m_linearVelocitySetpointFilter{ Regulators::Velocity::Linear::LAG_FILTER_K };
    LagFilter m_angularVelocitySetpointFilter{ Regulators::Velocity::Angular::LAG_FILTER_K };

    float m_targetLinearVelocity{};
    float m_targetAngularVelocity{};
};
