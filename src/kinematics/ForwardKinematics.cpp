#include "kinematics/ForwardKinematics.hpp"

#include "Constants.hpp"

#include "filters/MAFilter.hpp"

#include "state/Radians.hpp"
#include "state/Vector.hpp"

#include <optional>

ForwardKinematics::ForwardKinematics(Vec2 const& wheelAngles) : m_prevWheelAngles{ wheelAngles } {}

void ForwardKinematics::update(Vec2 const& wheelAngles, std::optional<float> heading,
                               std::optional<float> angularVelocity, float dt) {
    Radians const dWheelAngleLeft{ wheelAngles.x - m_prevWheelAngles.x };
    Radians const dWheelAngleRight{ wheelAngles.y - m_prevWheelAngles.y };

    float const dLeft = Chassis::WHEEL_RADIUS * dWheelAngleLeft.toFloat();
    float const dRight = Chassis::WHEEL_RADIUS * dWheelAngleRight.toFloat();
    float const d = (dLeft + dRight) / 2.0f;

    float const deltaTheta = (dRight - dLeft) / Chassis::AXLE_LENGTH;
    float const theta = heading.value_or(m_prevTheta + deltaTheta);

    m_state.position += Vec2::fromPolar(d, m_prevTheta + deltaTheta / 2.0f);
    m_state.angle = theta;
    m_state.angularVelocity =
        angularVelocity.value_or(m_angularVelocityFilter.update((theta - m_prevTheta) / dt, dt));

    Vec2 const velocity{ (m_state.position - m_prevPosition) / dt };
    m_state.velocity.x = m_velocityXFilter.update(velocity.x, dt);
    m_state.velocity.y = m_velocityYFilter.update(velocity.y, dt);

    m_state.wheelSpeeds.x = m_leftWheelSpeedFilter.update(dWheelAngleLeft.toFloat() / dt, dt);
    m_state.wheelSpeeds.y = m_rightWheelSpeedFilter.update(dWheelAngleRight.toFloat() / dt, dt);

    m_prevPosition = m_state.position;
    m_prevWheelAngles = wheelAngles;
    m_prevTheta = theta;
}
