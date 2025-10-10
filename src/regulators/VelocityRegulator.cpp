#include "regulators/VelocityRegulator.hpp"

#include "state/Radians.hpp"
#include "state/Vector.hpp"

#include <algorithm>

Vec2 VelocityRegulator::update(Vec2 const& currentVelocity, Radians currentAngle,
                               float batteryVoltage, float dt) {
    float const angleDifference = Radians{ currentVelocity.angle() + Constants::PI / 2.0f } -
                                  currentAngle;
    float const currentLinearVelocity = std::copysignf(currentVelocity.length(), angleDifference);

    float const filteredTargetLinearVelocity = m_linearVelocitySetpointFilter.update(
        m_targetLinearVelocity, dt);
    float const linearVelocityTargetVoltage = m_linearVelocityController.update(
        filteredTargetLinearVelocity, currentLinearVelocity, dt);

    float const angleError = m_targetAngle - currentAngle;
    float const angleControlTargetVoltage = m_angleController.update(angleError, 0.0f, dt);

    float const linearVelocityVoltageBudget = batteryVoltage *
                                              Regulators::Velocity::LINEAR_VELOCITY_VOLTAGE_BUDGET;
    float const linearVelocityVoltage = std::clamp(
        linearVelocityTargetVoltage, -linearVelocityVoltageBudget, linearVelocityVoltageBudget);

    float const angleVoltageBudget = batteryVoltage - linearVelocityVoltage;
    float const angleControlVoltage = std::clamp(angleControlTargetVoltage, -angleVoltageBudget,
                                                 angleVoltageBudget);

    float const leftOutput = linearVelocityVoltage - angleControlVoltage;
    float const rightOutput = linearVelocityVoltage + angleControlVoltage;

    // std::printf(">currentLinearVelocity:%.5f\n>targetLinearVelocity:%.5f\n>linearOutput:%.5f\n",
    //             currentLinearVelocity, filteredTargetLinearVelocity, linearVelocityVoltage);
    // std::printf(">currentAngle:%.5f\n>targetAngle:%.5f\n>angularOutput:%.5f\n",
    //             currentAngle.toFloat(), filteredTargetAngle, angleCurrent);
    // std::printf(">leftOutput:%.5f\n>rightOutput:%.5f\n", leftOutput, rightOutput);

    return { leftOutput, rightOutput };
}
