#include "regulators/VelocityRegulator.hpp"

#include "state/Radians.hpp"
#include "state/Vector.hpp"

#include <algorithm>

VelocityRegulator::VelocityRegulator(float dt) :
    m_linearVelocityController{
        { Regulators::Velocity::Linear::kS },
        { Regulators::Velocity::Linear::kV },
        { Regulators::Velocity::Linear::kA, Regulators::Velocity::Linear::FILTER_ALPHA, dt },
        { Regulators::Velocity::Linear::kP },
        { Regulators::Velocity::Linear::kI, Regulators::Velocity::Linear::MIN_INT,
          Regulators::Velocity::Linear::MAX_INT, dt }
    },
    m_anglularVelocityController{
        { Regulators::Velocity::Angular::kS },
        { Regulators::Velocity::Angular::kV },
        { Regulators::Velocity::Angular::kA, Regulators::Velocity::Angular::FILTER_ALPHA, dt },
        { Regulators::Velocity::Angular::kP },
        { Regulators::Velocity::Angular::kI, Regulators::Velocity::Angular::MIN_INT,
          Regulators::Velocity::Angular::MAX_INT, dt }
    },
    m_linearVelocityOutputFilter{ Regulators::Velocity::Linear::OUTPUT_FILTER_CUTOFF_FREQ, dt } {}

Vec2 VelocityRegulator::update(Vec2 const& currentVelocity, Radians currentAngle,
                               float currentAngularVelocity, float batteryVoltage) {
    float const angleDifference = Radians{ currentVelocity.angle() + Constants::PI / 2.0f } -
                                  currentAngle;
    float const currentLinearVelocity = std::copysignf(currentVelocity.length(), angleDifference);

    float const filteredTargetLinearVelocity =
        m_linearVelocitySetpointFilter.update(m_targetLinearVelocity);
    float const linearVelocityTargetVoltage = m_linearVelocityController.update(
        filteredTargetLinearVelocity, currentLinearVelocity);
    float const filteredLinearVelocityTargetVoltage =
        m_linearVelocityOutputFilter.update(linearVelocityTargetVoltage);

    float const filteredTargetAngularVelocity =
        m_angularVelocitySetpointFilter.update(m_targetAngularVelocity);
    float const anglularVelocityControlTargetVoltage = m_anglularVelocityController.update(
        filteredTargetAngularVelocity, currentAngularVelocity);

    float const linearVelocityVoltageBudget = batteryVoltage *
                                              Regulators::Velocity::LINEAR_VELOCITY_VOLTAGE_BUDGET;
    float const linearVelocityVoltage = std::clamp(filteredLinearVelocityTargetVoltage,
                                                   -linearVelocityVoltageBudget,
                                                   linearVelocityVoltageBudget);

    float const angularVelocityVoltageBudget = batteryVoltage - linearVelocityVoltage;
    float const angularVelocityControlVoltage = std::clamp(anglularVelocityControlTargetVoltage,
                                                           -angularVelocityVoltageBudget,
                                                           angularVelocityVoltageBudget);

    float const leftVoltage = linearVelocityVoltage - angularVelocityControlVoltage;
    float const rightVoltage = linearVelocityVoltage + angularVelocityControlVoltage;

    float leftOffsetVoltage = leftVoltage;
    float rightOffsetVoltage = rightVoltage * Regulators::Velocity::OFFSET;

    if (leftOffsetVoltage > batteryVoltage || leftOffsetVoltage < -batteryVoltage ||
        rightOffsetVoltage > batteryVoltage || rightOffsetVoltage < -batteryVoltage) {
        float const largerOffsetVoltage = std::max(std::fabsf(leftOffsetVoltage),
                                                   std::fabsf(rightOffsetVoltage));

        leftOffsetVoltage *= batteryVoltage / largerOffsetVoltage;
        rightOffsetVoltage *= batteryVoltage / largerOffsetVoltage;
    }

    return { leftOffsetVoltage, rightOffsetVoltage };
}
