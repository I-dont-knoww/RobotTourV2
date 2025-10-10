#include "regulators/CurrentRegulator.hpp"

#include "Constants.hpp"

#include "drivers/Motors.hpp"

#include "state/Vector.hpp"

#include <algorithm>
#include <cmath>

Vec2 CurrentRegulator::update(Vec2 const& wheelSpeeds, float batteryVoltage, float dt) {
    using Regulators::Current::KV;
    using Regulators::Current::MAX_CURRENT;
    using Regulators::Current::RESISTANCE;

    Vec2 const predictedCurrent = m_targetVoltages / RESISTANCE - wheelSpeeds / (KV * RESISTANCE);

    Vec2 const voltageMin = -MAX_CURRENT * RESISTANCE + wheelSpeeds / KV;
    Vec2 const voltageMax = MAX_CURRENT * RESISTANCE + wheelSpeeds / KV;

    Vec2 currentLimitedVoltages{};

    if (predictedCurrent.x > MAX_CURRENT) currentLimitedVoltages.x = voltageMax.x;
    else if (predictedCurrent.x < -MAX_CURRENT) currentLimitedVoltages.x = voltageMin.x;
    else currentLimitedVoltages.x = m_targetVoltages.x;

    if (predictedCurrent.y > MAX_CURRENT) currentLimitedVoltages.y = voltageMax.y;
    else if (predictedCurrent.y < -MAX_CURRENT) currentLimitedVoltages.y = voltageMin.y;
    else currentLimitedVoltages.y = m_targetVoltages.y;

    currentLimitedVoltages.transform(
        [=](float x) { return std::clamp(x, -batteryVoltage, batteryVoltage); });

    return currentLimitedVoltages / batteryVoltage * static_cast<float>(Drivers::Motors::MAX_POWER);
}
