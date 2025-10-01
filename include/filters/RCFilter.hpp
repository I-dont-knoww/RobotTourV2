#pragma once

class RCFilter {
public:
    RCFilter(float cutoff);

    float update(float input, float dt);

private:
    float const m_tau{};
    
    float m_prevOutput = 0.0f;
};
