#pragma once

#include "path/Compiler.hpp"
#include "path/Path.hpp"

#include "state/Vector.hpp"

#include <array>

namespace Competition {
    using namespace Compiler::Tokens;

    inline constexpr float TARGET_TIME = 65.0f;
    inline constexpr auto COMMANDS = std::to_array<Compiler::Command>({
        // clang-format off

        FIRST_MOVE,

        moveby(UP),
        moveby(RIGHT),
        moveby(UP),
        moveby(0.7f * UP),
        moveby(0.7f * DOWN) & REVERSE,
        moveby(DOWN),
        moveby(LEFT),
        moveby(LEFT),
        moveby(RIGHT),
        moveby(0.7f * UP),
        moveby(0.7f * DOWN) & REVERSE,
        moveby(DOWN),
        moveby(LEFT),
        moveby(LEFT),
        moveby(UP),
        moveby(LEFT),
        moveby(0.7f * DOWN),
        moveby(0.7f * UP) & REVERSE,
        moveby(UP),
        moveby(UP),
        moveby(RIGHT),

        moveby(DOWN) & LAST_MOVE

        // clang-format on
    });

    inline constexpr auto PATH = Compiler::compile(COMMANDS);
    inline constexpr auto TARGET_TIMES = Compiler::getTargetTimes(COMMANDS, PATH, TARGET_TIME);
    inline constexpr auto BRUH = Compiler::getTurnTimes(PATH);
    inline constexpr Vec2 DESTINATION = Compiler::getDestination(PATH) / SQUARE_SIZE;
}
