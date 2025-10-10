#include "managers/Linear.hpp"

#include "Constants.hpp"

#include "state/Vector.hpp"

#include <algorithm>
#include <cmath>

float Linear::update(Vec2 const& currentPosition, float currentTime, float dt) {
    float const distance = (m_reverse ? -1.0f : 1.0f) *
                           std::copysignf((m_targetPosition - currentPosition).length(),
                                          Vec2::dot(m_targetPosition - m_startPosition,
                                                    m_targetPosition - currentPosition));

    float const positionControlSpeed = m_positionController.update(distance, 0.0f, dt);

    float const timeLeft = m_targetTime - currentTime;
    if (timeLeft <= Manager::Speed::GIVE_UP_TIME) {
        if (m_stop) return positionControlSpeed;
        else return Manager::Speed::MAX_SPEED;
    }

    float const targetSpeed = distance / timeLeft;

    float motorSpeed;
    if (m_stop) {
        if (m_reverse) motorSpeed = std::max(positionControlSpeed, targetSpeed);
        else motorSpeed = std::min(positionControlSpeed, targetSpeed);
    } else motorSpeed = targetSpeed;

    return motorSpeed;
}
