#pragma once

class VController {
public:
    VController(float kV) : m_kV{ kV } {}

    float update(float setpoint, float, float) { return m_kV * setpoint; }

private:
    float m_kV{};
};
