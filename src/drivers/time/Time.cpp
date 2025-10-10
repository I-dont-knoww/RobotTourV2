#include "drivers/Time.hpp"

#include "hardware/timer.h"

void Time::reset() {
    m_startTime = time_us_32();
    m_elapsedTime = 0.0f;
}

void Time::update() {
    uint32_t const currentTime = time_us_32();
    m_elapsedTime = static_cast<float>(currentTime - m_startTime) * 1.0e-6f;
}
