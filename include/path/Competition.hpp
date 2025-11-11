#pragma once

#include "path/Compiler.hpp"
#include "path/Path.hpp"

#include "state/Vector.hpp"

#include <array>

namespace Competition {
    using namespace Compiler::Tokens;

    inline constexpr float TARGET_TIME = 50.0f;
    inline constexpr auto COMMANDS = std::to_array<Compiler::Command>({
        // clang-format off

        FIRST_MOVE,

        moveby(UP + LEFT),
        moveby(UP),
        moveby(RIGHT),
        moveby(UP),
        moveby(LEFT),
        moveby(UP),
        moveby(RIGHT),
        moveby(RIGHT),
        moveby(RIGHT),
        moveby(DOWN),
        moveby(LEFT),
        moveby(DOWN),
        moveby(RIGHT),
        moveby(DOWN),

        moveby(LEFT + DOWN) & LAST_MOVE & OFFSET_ONCE(1.5f * LEFT + 0.75f * DOWN),

        // clang-format on
    });

    inline constexpr auto PATH = Compiler::compile(COMMANDS);
    inline constexpr auto TARGET_TIMES = Compiler::getTargetTimes(COMMANDS, PATH, TARGET_TIME);
    inline constexpr Vec2 DESTINATION = Compiler::getDestination(PATH) / SQUARE_SIZE;
}
