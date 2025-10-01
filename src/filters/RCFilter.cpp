#include "filters/RCFilter.hpp"

#include "Constants.hpp"

RCFilter::RCFilter(float cutoff) : m_tau{ 1.0f / (2.0f * Constants::PI * cutoff) } {}

float RCFilter::update(float input, float dt) {
    float const output = (dt / (dt + m_tau)) * input + (m_tau / (dt + m_tau)) * m_prevOutput;
    m_prevOutput = output;
    return output;
}
