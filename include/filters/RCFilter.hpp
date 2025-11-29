#pragma once

class RCFilter {
public:
    RCFilter(float cutoff, float dt);

    float update(float input);

private:
    float const m_tau{};
    float const m_coefficient1{};
    float const m_coefficient2{};
    
    float m_prevOutput = 0.0f;
};
