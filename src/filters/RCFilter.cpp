#include "filters/RCFilter.hpp"

#include "Constants.hpp"

static constexpr float K = 1.0f / 2.0f * Constants::PI;

RCFilter::RCFilter(float cutoff, float dt)
    : m_coefficient1{ dt / (dt + (K / cutoff)) },
      m_coefficient2{ (K / cutoff) / (dt + (K / cutoff)) } {}

float RCFilter::update(float input) {
    float const output = m_coefficient1 * input + m_coefficient2 * m_prevOutput;
    m_prevOutput = output;

    return output;
}
