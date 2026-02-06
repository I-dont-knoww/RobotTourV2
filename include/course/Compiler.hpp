#pragma once

#include "course/Assigner.hpp"
#include "course/Converter.hpp"
#include "course/Course.hpp"
#include "course/Extender.hpp"
#include "course/Smoothener.hpp"

#include <array>

namespace Course {
    template <std::array path, float targetTime>
    constexpr auto compile() {
        static constexpr auto converted = Converter::convert<path>();
        static constexpr auto extended = Extender::extend<converted>();
        static constexpr auto smoothened = Smoothener::smoothen<extended>();
        static constexpr auto velocityLimited = Assigner::assignVelocityLimits<smoothened>();
        static constexpr auto distanceLeft = Assigner::assignDistanceLeft(velocityLimited);
        static constexpr auto targetTimes = Assigner::assignTargetTimes(distanceLeft, targetTime);

        return targetTimes;
    }
}
