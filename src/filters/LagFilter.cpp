// http://my.execpc.com/~steidl/robotics/first_order_lag_filter.html

#include "filters/LagFilter.hpp"

#include "Constants.hpp"

#include <cmath>

LagFilter::LagFilter(float k) : m_k{ k } {}

float LagFilter::update(float input) {
    float const output = m_k * input + (1.0f - m_k) * m_prevOutput;
    m_prevOutput = output;

    return output;
}
