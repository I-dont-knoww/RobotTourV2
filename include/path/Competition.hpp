#pragma once

#include "path/Compiler.hpp"
#include "path/Path.hpp"

#include "state/Vector.hpp"

#include <array>

namespace Competition {
    using namespace Compiler::Tokens;

    inline constexpr bool FAIL_RUN = false;
    inline constexpr float TARGET_TIME = 0.0f;

    inline constexpr auto COMMANDS = std::to_array<Compiler::Command>({
        // clang-format off

        moveby(0.8f * UP),
        moveby(0.6f * LEFT),

        // moveby(UP),
        // moveby(LEFT),
        // moveby(DOWN),
        // moveby(RIGHT)

        // clang-format on
    });

    inline constexpr auto PATH = Compiler::compile(COMMANDS);
    inline constexpr auto TARGET_TIMES = Compiler::getTargetTimes(COMMANDS, PATH, TARGET_TIME);
    inline constexpr auto TURN_TIMES = Compiler::TargetTime::getTurnTimes(PATH);
    inline constexpr Vec2 DESTINATION = Compiler::getDestination(PATH)/*  / SQUARE_SIZE */;
}
