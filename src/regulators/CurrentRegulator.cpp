#include "regulators/CurrentRegulator.hpp"

#include "Constants.hpp"

#include "drivers/Motors.hpp"

#include "state/Vector.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <optional>

static constexpr size_t SCALE_VALUE_COUNT = 5;

std::optional<float> divide(float a, float b) {
    if (b == 0.0f) return std::nullopt;
    return a / b;
}

std::array<std::optional<float>, SCALE_VALUE_COUNT> getScaleValues(Vec2 const& mins,
                                                                   Vec2 const& maxes,
                                                                   Vec2 const& targets) {
    return { divide(maxes.x, targets.x), divide(maxes.y, targets.y), divide(mins.x, targets.x),
             divide(mins.y, targets.y), 1.00f };
}

auto getScaledVectors(std::array<std::optional<float>, SCALE_VALUE_COUNT> const& scaleValues,
                     Vec2 const& targets) {
    std::array<std::optional<Vec2>, SCALE_VALUE_COUNT> scaledVectors{};
    for (size_t i = 0; i < scaledVectors.size(); ++i)
        scaledVectors[i] = scaleValues[i].transform([&](float x) { return targets * x; });
    return scaledVectors;
}

Vec2 scale(Vec2 const& mins, Vec2 const& maxes, Vec2 const& targets) {
    auto const scaleValues = getScaleValues(mins, maxes, targets);
    auto const scaledVectors = getScaledVectors(scaleValues, targets);

    std::optional<size_t> scaleIndex{};
    for (size_t i = 0; i < scaleValues.size(); ++i) {
        if (!scaledVectors[i].has_value()) continue;
        Vec2 const& scaledVector = *scaledVectors[i];

        if (scaledVector.x < mins.x || scaledVector.x > maxes.x || scaledVector.y < mins.y ||
            scaledVector.y > maxes.y)
            continue;

        if (!scaleIndex.has_value() ||
            std::fabsf(*scaleValues[i] - 1.0f) < std::fabsf(*scaleValues[*scaleIndex] - 1.0f))
            scaleIndex = i;
    }

    if (!scaleIndex.has_value())
        return Vec2::transform(
            [](float voltage, float min, float max) { return std::clamp(voltage, min, max); },
            targets, mins, maxes);
    else return *scaledVectors[*scaleIndex];
}

Vec2 CurrentRegulator::update(Vec2 const& wheelSpeeds, float batteryVoltage, float dt) {
    using Regulators::Current::KV;
    using Regulators::Current::MAX_CURRENT;
    using Regulators::Current::RESISTANCE;

    Vec2 const voltageMins = -MAX_CURRENT * RESISTANCE + wheelSpeeds / KV;
    Vec2 const voltageMaxes = MAX_CURRENT * RESISTANCE + wheelSpeeds / KV;

    Vec2 currentLimitedVoltages = scale(voltageMins, voltageMaxes, m_targetVoltages);

    return currentLimitedVoltages / batteryVoltage * static_cast<float>(Drivers::Motors::MAX_POWER);
}
