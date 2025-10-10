#pragma once

#include <cstdint>

class Time {
public:
    Time() = default;

    void reset();
    void update();

    float elapsed() const { return m_elapsedTime; }

private:
    uint32_t m_startTime{};
    float m_elapsedTime{};
};
