#include "fusion/Fusion.hpp"

#include "Constants.hpp"

Fusion::Fusion(float dt) :
    m_gyroscopeFilter{ Kinematics::Forward::GYROSCOPE_CUTOFF_FREQ, dt }, m_dt{ dt } {}

float Fusion::update(float angularVelocity) {
    float const filteredAngularVelocity = m_gyroscopeFilter.update(angularVelocity);
    
    m_heading += filteredAngularVelocity * m_dt;
    return m_heading;
}
