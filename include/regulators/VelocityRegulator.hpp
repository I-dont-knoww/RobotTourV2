#pragma once

#include "Constants.hpp"

#include "control/Controller.hpp"
#include "control/feedfoward/AController.hpp"
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

    void move(float targetLinearVelocity, Radians targetAngularVelocity) {
        m_targetLinearVelocity = targetLinearVelocity;
        m_targetAngularVelocity = targetAngularVelocity;
    }

    Vec2 update(Vec2 const& actualVelocity, Radians angle, float actualAngularVelocity, float dt);

private:
    // Controller<PController, IController, DController> m_linearVelocityController{
    //     { Regulators::Velocity::Linear::kP },
    //     { Regulators::Velocity::Linear::kI, Regulators::Velocity::Linear::MIN_INT,
    //       Regulators::Velocity::Linear::MAX_INT },
    //     { Regulators::Velocity::Linear::kD, Regulators::Velocity::Linear::CUTOFF_FREQUENCY }
    // };
    Controller<VController, AController, PController> m_linearVelocityController{
        { Regulators::Velocity::Linear::kV },
        { Regulators::Velocity::Linear::kA, Regulators::Velocity::Linear::CUTOFF_FREQUENCY },
        { Regulators::Velocity::Linear::kP }
    };
    // Controller<PController, IController> m_linearVelocityIntegrator{
    //     { Regulators::Velocity::Linear::kPIntegrator }, { 1.0f, -1.0f, 1.0f }
    // };
    // Controller<PController, DController> m_linearVelocityController{
    //     { Regulators::Velocity::Linear::kPControl },
    //     { Regulators::Velocity::Linear::kD, Regulators::Velocity::Linear::CUTOFF_FREQUENCY }
    // };

    // Controller<PController, IController, DController> m_angularVelocityController{
    //     { Regulators::Velocity::Angular::kP },
    //     { Regulators::Velocity::Angular::kI, Regulators::Velocity::Angular::MIN_INT,
    //       Regulators::Velocity::Angular::MAX_INT },
    //     { Regulators::Velocity::Angular::kD, Regulators::Velocity::Angular::CUTOFF_FREQUENCY }
    // };
    Controller<VController, AController, PController, DController> m_angularVelocityController {
        { Regulators::Velocity::Angular::kV },
        { Regulators::Velocity::Angular::kA, Regulators::Velocity::Angular::CUTOFF_FREQUENCY },
        { Regulators::Velocity::Angular::kP },
        { Regulators::Velocity::Angular::kD, Regulators::Velocity::Angular::ALPHA }
    };

    LagFilter linearVelocitySetpointFilter{ Regulators::Velocity::Linear::LAG_FILTER_K };
    LagFilter angularVelocitySetpointFilter{ Regulators::Velocity::Angular::LAG_FILTER_K };

    float m_targetLinearVelocity{};
    Radians m_targetAngularVelocity{};
};
