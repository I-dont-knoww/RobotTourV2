#pragma once

class PController {
public:
    PController(float kP) : m_kP{ kP } {}

    float update(float setpoint, float measurement, float) {
        return (setpoint - measurement) * m_kP;
    }

private:
    float m_kP{};
};
