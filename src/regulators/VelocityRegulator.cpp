#include "regulators/VelocityRegulator.hpp"

#include "state/Radians.hpp"
#include "state/Vector.hpp"

#include <algorithm>

Vec2 VelocityRegulator::update(Vec2 const& currentVelocity, Radians currentAngle,
                               float currentAngularVelocity, float batteryVoltage, float dt) {
    float const angleDifference = Radians{ currentVelocity.angle() + Constants::PI / 2.0f } -
                                  currentAngle;
    float const currentLinearVelocity = std::copysignf(currentVelocity.length(), angleDifference);

    float const filteredTargetLinearVelocity = m_linearVelocitySetpointFilter.update(
        m_targetLinearVelocity, dt);
    float const linearVelocityTargetVoltage = m_linearVelocityController.update(
        filteredTargetLinearVelocity, currentLinearVelocity, dt);

    float const filteredTargetAngularVelocity = m_angularVelocitySetpointFilter.update(
        m_targetAngularVelocity, dt);
    float const anglularVelocityControlTargetVoltage = m_anglularVelocityController.update(
        filteredTargetAngularVelocity, currentAngularVelocity, dt);

    float const linearVelocityVoltageBudget = batteryVoltage *
                                              Regulators::Velocity::LINEAR_VELOCITY_VOLTAGE_BUDGET;
    float const linearVelocityVoltage = std::clamp(
        linearVelocityTargetVoltage, -linearVelocityVoltageBudget, linearVelocityVoltageBudget);

    float const angularVelocityVoltageBudget = batteryVoltage - linearVelocityVoltage;
    float const angularVelocityControlVoltage = std::clamp(anglularVelocityControlTargetVoltage,
                                                           -angularVelocityVoltageBudget,
                                                           angularVelocityVoltageBudget);

    float const leftVoltage = linearVelocityVoltage - angularVelocityControlVoltage;
    float const rightVoltage = linearVelocityVoltage + angularVelocityControlVoltage;

    float leftOffsetVoltage = leftVoltage;
    float rightOffsetVoltage = rightVoltage * Regulators::Velocity::OFFSET;
    if (leftOffsetVoltage > batteryVoltage || rightOffsetVoltage > batteryVoltage) {
        float const largerOffsetVoltage = std::max(leftOffsetVoltage, rightOffsetVoltage);
        leftOffsetVoltage *= batteryVoltage / largerOffsetVoltage;
        rightOffsetVoltage *= batteryVoltage / largerOffsetVoltage;
    }

    // std::printf(">currentLinearVelocity:%.5f\n>targetLinearVelocity:%.5f\n>linearOutput:%.5f\n",
    //             currentLinearVelocity, filteredTargetLinearVelocity, linearVelocityVoltage);
    // std::printf(">currentAngularVelocity:%.5f\n>targetAngularVelocity:%.5f\n>angularOutput:%.5f\n",
    //             currentAngularVelocity, filteredTargetAngularVelocity,
    //             angularVelocityControlVoltage);
    // std::printf(">leftVoltage:%.5f\n>rightVoltage:%.5f\n", leftVoltage, rightVoltage);

    return { leftOffsetVoltage, rightOffsetVoltage };
}
