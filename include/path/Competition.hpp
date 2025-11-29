#pragma once

#include "path/Compiler.hpp"
#include "path/Path.hpp"

#include "state/Vector.hpp"

#include <array>

namespace Competition {
    using namespace Compiler::Tokens;

    inline constexpr float TARGET_TIME = 5.0f;
    inline constexpr auto COMMANDS = std::to_array<Compiler::Command>({
        // clang-format off

        moveby(50.0f * UP) & CENTIMETERS

        // clang-format on
    });

    // 8.3cm

    inline constexpr auto PATH = Compiler::compile(COMMANDS);
    inline constexpr auto TARGET_TIMES = Compiler::getTargetTimes(COMMANDS, PATH, TARGET_TIME);
    inline constexpr auto TURN_TIMES = Compiler::TargetTime::getTurnTimes(PATH);
    inline constexpr Vec2 DESTINATION = Compiler::getDestination(PATH) / SQUARE_SIZE;
}
