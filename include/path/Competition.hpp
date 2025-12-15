#pragma once

#include "path/Compiler.hpp"
#include "path/Path.hpp"

#include "state/Vector.hpp"

#include <array>

namespace Competition {
    using namespace Compiler::Tokens;

    inline constexpr float TARGET_TIME = 67.0f;
    inline constexpr auto COMMANDS = std::to_array<Compiler::Command>({
        // clang-format off

        FIRST_MOVE,

        moveby(UP),
        moveby(RIGHT),
        moveby(DOWN),
        moveby(RIGHT),
        moveby(0.7f * UP),
        moveby(0.7f * DOWN) & REVERSE,
        moveby(LEFT),
        moveby(UP),
        moveby(LEFT),
        moveby(LEFT),
        moveby(DOWN),
        moveby(0.7f * LEFT),
        moveby(0.7f * RIGHT) & REVERSE,
        moveby(UP),
        moveby(UP),
        moveby(LEFT),
        moveby(RIGHT),
        moveby(UP),
        moveby(RIGHT),
        moveby(RIGHT),
        moveby(0.7f * RIGHT),
        moveby(0.7f * LEFT) & REVERSE,
        moveby(LEFT),

        moveby(LEFT) & LAST_MOVE

        // clang-format on
    });

    // 8.3cm

    inline constexpr auto PATH = Compiler::compile(COMMANDS);
    inline constexpr auto TARGET_TIMES = Compiler::getTargetTimes(COMMANDS, PATH, TARGET_TIME);
    inline constexpr auto TURN_TIMES = Compiler::TargetTime::getTurnTimes(PATH);
    inline constexpr Vec2 DESTINATION = Compiler::getDestination(PATH) / SQUARE_SIZE;
}
