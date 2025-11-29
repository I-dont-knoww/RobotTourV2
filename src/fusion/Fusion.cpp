#include "fusion/Fusion.hpp"

Fusion::Fusion(float dt) : m_dt{ dt } {}

float Fusion::update(float angularVelocity) {
    m_heading += angularVelocity * m_dt;
    return m_heading;
}
