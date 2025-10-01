#include "regulators/VelocityRegulator.hpp"

#include "state/Radians.hpp"
#include "state/Vector.hpp"

#include <algorithm>

#include <cstdio>

Vec2 VelocityRegulator::update(Vec2 const& actualVelocity, Radians angle,
                               float actualAngularVelocity, float dt) {
    float const angleDifference = Radians{ actualVelocity.angle() + Constants::PI / 2.0f } - angle;
    float const actualLinearVelocity = std::copysignf(actualVelocity.length(), angleDifference);

    float const filteredTargetLinearVelocity = linearVelocitySetpointFilter.update(
        m_targetLinearVelocity, dt);
    float const filteredTargetAngularVelocity = angularVelocitySetpointFilter.update(
        m_targetAngularVelocity, dt);

    // float const linearVelocityPower = m_linearVelocityIntegrator.update(
    //     m_linearVelocityController.update(filteredTargetLinearVelocity, actualLinearVelocity,
    //     dt), 0.0f, dt);
    float const linearVelocityPower = m_linearVelocityController.update(
        filteredTargetLinearVelocity, actualLinearVelocity, dt);
    float const angularVelocityPower = m_angularVelocityController.update(
        filteredTargetAngularVelocity, actualAngularVelocity, dt);

    float const linearVelocityCurrentFraction = std::clamp(
        linearVelocityPower, -Regulators::Velocity::LINEAR_VELOCITY_CURRENT_BUDGET,
        Regulators::Velocity::LINEAR_VELOCITY_CURRENT_BUDGET);

    float const angularVelocityCurrentBudget = 1.0f - linearVelocityCurrentFraction;
    float const angularVelocityCurrentFraction = std::clamp(
        angularVelocityPower, -angularVelocityCurrentBudget, angularVelocityCurrentBudget);

    float const linearVelocityCurrent = linearVelocityCurrentFraction *
                                        Regulators::Velocity::MAX_CURRENT;
    float const angularVelocityCurrent = angularVelocityCurrentFraction *
                                         Regulators::Velocity::MAX_CURRENT;

    float const leftOutput = linearVelocityCurrent - angularVelocityCurrent;
    float const rightOutput = linearVelocityCurrent + angularVelocityCurrent;

    std::printf(">currentLinearVelocity:%.5f\n>targetLinearVelocity:%.5f\n>linearOutput:%.5f\n",
                actualLinearVelocity, filteredTargetLinearVelocity, linearVelocityCurrent);
    std::printf(">currentAngularVelocity:%.5f\n>targetAngularVelocity:%.5f\n>angularOutput:%.5f\n",
                actualAngularVelocity, filteredTargetAngularVelocity, angularVelocityCurrent);
    std::printf(">leftOutput:%.5f\n>rightOutput:%.5f\n", leftOutput, rightOutput);

    return { leftOutput, rightOutput };
}
