// https://medium.com/@vikramaditya.nishant/using-field-oriented-control-to-make-the-perfect-robot-drivetrain-d57a45fb14ee

#pragma once

#include "Constants.hpp"

#include "control/Controller.hpp"
#include "control/pid/IController.hpp"
#include "control/pid/PController.hpp"

#include "drivers/Motors.hpp"

#include "filters/LagFilter.hpp"
#include "filters/RCFilter.hpp"

#include <algorithm>

class CurrentRegulator {
public:
    CurrentRegulator(Motors& motors, float batteryVoltage);

    void update(Vec2 const& wheelSpeeds, float batteryVoltage, float dt);

    void spin(float current) { spin(current, current); }
    void spin(float targetLeftCurrent, float targetRightCurrent) {
        m_targetLeftCurrent = targetLeftCurrent;
        m_targetRightCurrent = targetRightCurrent;
    }

private:
    Controller<PController, IController> m_leftController;
    Controller<PController, IController> m_rightController;

    RCFilter m_leftCurrentFilter{ Regulators::Current::RC_FILTER_CUTOFF_FREQUENCY };
    RCFilter m_rightCurrentFilter{ Regulators::Current::RC_FILTER_CUTOFF_FREQUENCY };

    LagFilter m_leftTargetFilter{ Regulators::Current::LAG_FILTER_K };
    LagFilter m_rightTargetFilter{ Regulators::Current::LAG_FILTER_K };

    Motors& m_motors;

    float m_targetLeftCurrent = 0.0f;
    float m_targetRightCurrent = 0.0f;
};
