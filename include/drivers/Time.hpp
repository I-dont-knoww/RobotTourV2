#pragma once

#include <cstdint>

class Time {
public:
    Time() = default;

    void reset();
    void update();

    float elapsed() const { return m_elapsedTime; }
    float delta() const { return m_deltaTime; }

private:
    uint32_t m_startTime{};
    uint32_t m_prevTime{};

    float m_elapsedTime{};
    float m_deltaTime{};
};
