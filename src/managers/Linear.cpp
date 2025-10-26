#include "managers/Linear.hpp"

#include "Constants.hpp"

#include "state/Vector.hpp"

#include <algorithm>
#include <cmath>

float Linear::update(Vec2 const& currentPosition, Vec2 const& currentVelocity,
                     float angularVelocity, float currentTime, float dt) {
    float const distance = (m_reverse ? -1.0f : 1.0f) *
                           std::copysignf((m_targetPosition - currentPosition).length(),
                                          Vec2::dot(m_targetPosition - m_startPosition,
                                                    m_targetPosition - currentPosition));
    float const positionControlSpeed = m_positionController.update(distance, 0.0f, dt);

    float linearSpeed{};

    float const timeLeft = m_targetTime - currentTime;
    if (timeLeft <= Manager::Speed::GIVE_UP_TIME) {
        if (m_stop) linearSpeed = positionControlSpeed;
        else linearSpeed = Manager::Speed::MAX_SPEED;
    } else {
        float const targetSpeed = distance / timeLeft;

        if (m_stop) {
            if (m_reverse) linearSpeed = std::max(positionControlSpeed, targetSpeed);
            else linearSpeed = std::min(positionControlSpeed, targetSpeed);
        } else linearSpeed = targetSpeed;
    }

    if (angularVelocity != 0.0f) {
        float const centripetalForceSpeed =
            std::fabsf(Manager::Speed::MAX_CENTRIPETAL / (Chassis::MASS * angularVelocity));
        linearSpeed = std::clamp(linearSpeed, -centripetalForceSpeed, centripetalForceSpeed);
    }
    linearSpeed = std::clamp(linearSpeed, -Manager::Speed::MAX_SPEED, Manager::Speed::MAX_SPEED);

    return linearSpeed;
}
