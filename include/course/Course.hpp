#pragma once

#include "state/Vector.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>

namespace Course {
    struct Segment {
        Vec2 position{};
        float velocityLimit{};
        float distanceToEnd{};
    };

    struct Route {
        using Flag = uint8_t;
        static constexpr Flag NO_FLAGS = 0b00000000;
        static constexpr Flag REVERSE = 0b00000001;

        size_t beginIndex{};
        size_t endIndex{};

        float targetTime{};
        Flag flags{};
    };

    template <size_t SegmentCount, size_t RouteCount>
    struct Course {
        std::array<Segment, SegmentCount> segments{};
        std::array<Route, RouteCount> routes{};
    };
}
