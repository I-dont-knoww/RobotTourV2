#pragma once

#include "path/Compiler.hpp"
#include "path/Path.hpp"

#include "state/Vector.hpp"

#include <array>

namespace Competition {
    using namespace Compiler::Tokens;

    inline constexpr float TARGET_TIME = 6.0f;
    inline constexpr auto PATH = Compiler::compile(std::to_array<Compiler::Command>({
        // clang-format off

        // moveby(0.0f * UP) & CENTIMETERS & STOP,
        // moveby(10.0f * LEFT) & CENTIMETERS,

        moveby(10.0f * UP) & CENTIMETERS & STOP,
        moveby(40.0f * RIGHT) & CENTIMETERS
        
        // moveby(UP) & STOP,
        // moveby(LEFT) & STOP,
        // moveby(DOWN) & STOP,
        // moveby(RIGHT) & STOP,

        // clang-format on
    }));

    inline constexpr auto TARGET_TIMES = Compiler::getTargetTimes(PATH, TARGET_TIME);
    inline constexpr Vec2 DESTINATION = Compiler::getDestination(PATH);
}
