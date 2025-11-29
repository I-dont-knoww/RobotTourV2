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
    CurrentRegulator() = default;

    Vec2 update(Vec2 const& wheelSpeeds, float batteryVoltage);

    void setTargetVoltage(Vec2 const& targetVoltages) { m_targetVoltages = targetVoltages; }

private:
    Vec2 m_targetVoltages{ 0.0f, 0.0f };
};
