#include "managers/Linear.hpp"

#include "Constants.hpp"

#include "state/Vector.hpp"

#include <algorithm>
#include <cmath>

float Movement::update(Vec2 const& currentPosition, float currentTime, float dt) {
    float const reverse = m_reverse ? -1.0f : 1.0f;

    float const distance = std::copysignf(
        (m_targetPosition - currentPosition).length(),
        Vec2::dot(m_targetPosition - m_startPosition, m_targetPosition - currentPosition));

    float const positionControlSpeed = std::max(3.0f,
                                                m_positionController.update(distance, 0.0f, dt));

    float const timeLeft = m_targetTime - currentTime;
    if (timeLeft <= Manager::Speed::GIVE_UP_TIME) {
        if (m_stop) return positionControlSpeed;
        else return Manager::Speed::MAX_SPEED;
    }

    float const targetSpeed = distance / timeLeft;

    float motorSpeed;
    if (m_stop) motorSpeed = std::min(positionControlSpeed, targetSpeed);
    else motorSpeed = targetSpeed;

    // std::printf(">distance:%.5f\n>targetSpeed:%.5f\n>output:%.5f\n", distance, targetSpeed,
    //             motorSpeed);
    // std::printf(">targetPosition:%.5f:%.5f|xy\n", m_targetPosition.x, m_targetPosition.y);
    // std::printf(">currentPosition:%.5f:%.5f|xy\n", currentPosition.x, currentPosition.y);

    return motorSpeed;
}
