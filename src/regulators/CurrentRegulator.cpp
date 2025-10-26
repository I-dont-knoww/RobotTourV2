#include "regulators/CurrentRegulator.hpp"

#include "Constants.hpp"

#include "drivers/Motors.hpp"

#include "state/Vector.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <optional>

Vec2 scale(Vec2 const& targets, Vec2 const& mins, Vec2 const& maxes) {
    std::array const scaleValues{ (maxes.x / targets.x), (maxes.y / targets.y),
                                  (mins.x / targets.x), (mins.y / targets.y), 1.00f };

    std::array<Vec2, scaleValues.size()> scaledVectors{};
    for (size_t i = 0; i < scaledVectors.size(); ++i) scaledVectors[i] = targets * scaleValues[i];

    std::optional<size_t> scaleIndex{};
    for (size_t i = 0; i < scaleValues.size(); ++i) {
        if (scaledVectors[i].x < mins.x || scaledVectors[i].x > maxes.x ||
            scaledVectors[i].y < mins.y || scaledVectors[i].y > maxes.y)
            continue;

        if (!scaleIndex.has_value() ||
            std::fabsf(scaleValues[i] - 1.0f) < std::fabsf(scaleValues[*scaleIndex] - 1.0f))
            scaleIndex = i;
    }

    if (!scaleIndex.has_value())
        return Vec2::transform(
            [](float voltage, float min, float max) { return std::clamp(voltage, min, max); },
            targets, mins, maxes);
    else return scaledVectors[*scaleIndex];
}

Vec2 CurrentRegulator::update(Vec2 const& wheelSpeeds, float batteryVoltage, float dt) {
    using Regulators::Current::KV;
    using Regulators::Current::MAX_CURRENT;
    using Regulators::Current::RESISTANCE;

    Vec2 const voltageMins = -MAX_CURRENT * RESISTANCE + wheelSpeeds / KV;
    Vec2 const voltageMaxes = MAX_CURRENT * RESISTANCE + wheelSpeeds / KV;

    Vec2 currentLimitedVoltages = scale(m_targetVoltages, voltageMins, voltageMaxes);

    return currentLimitedVoltages / batteryVoltage * static_cast<float>(Drivers::Motors::MAX_POWER);
}
