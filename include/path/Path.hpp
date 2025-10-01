#pragma once

#include "state/Vector.hpp"

#include <cstdint>

using uint = unsigned int;

struct Path {
    static constexpr uint NO_FLAGS{ 0b00000000 };
    static constexpr uint REVERSE{ 0b00000001 };
    static constexpr uint STOP{ 0b00000010 };
    static constexpr uint ACCURATE{ 0b00000100 };

    Vec2 position{};
    uint flags{};
};
