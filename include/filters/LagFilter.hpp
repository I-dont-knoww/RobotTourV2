#pragma once

class LagFilter {
public:
    LagFilter(float k);

    float update(float input);

private:
    float const m_k{};

    float m_prevOutput = 0.0f;
};
