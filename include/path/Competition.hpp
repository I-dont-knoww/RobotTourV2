#pragma once

#include "path/Compiler.hpp"
#include "path/Path.hpp"

#include "state/Vector.hpp"

#include <array>

namespace Competition {
    using namespace Compiler::Tokens;

    inline constexpr bool FAIL_RUN = false;
    inline constexpr float TARGET_TIME = 74.0f;

    inline constexpr auto COMMANDS = std::to_array<Compiler::Command>({
        // clang-format off

        FIRST_MOVE,

        moveby(UP),
        moveby(RIGHT),
        moveby(RIGHT),
        moveby(UP),
        moveby(UP),
        moveby(LEFT),
        moveby(LEFT),
        moveby(LEFT),
        moveby(0.7f * UP),
        moveby(0.7f * DOWN) & REVERSE,
        moveby(RIGHT),
        moveby(DOWN),
        moveby(UP),
        moveby(0.7f * LEFT),
        moveby(0.7f * RIGHT) & REVERSE,
        moveby(DOWN),
        moveby(LEFT),
        moveby(DOWN),
        moveby(DOWN),
        moveby(RIGHT),
        moveby(RIGHT),
        moveby(LEFT),
        moveby(LEFT),
        moveby(0.7f * UP),
        moveby(0.7f * DOWN) & REVERSE,
        moveby(RIGHT),
        moveby(RIGHT),
        moveby(RIGHT),
        moveby(LEFT),
        moveby(LEFT),
        moveby(UP),
        moveby(RIGHT),
        moveby(RIGHT),
        moveby(UP),
        moveby(UP),
        moveby(0.7f * LEFT),
        moveby(0.7f * RIGHT) & REVERSE,
        moveby(UP),

        moveby(LEFT) & OFFSET_ONCE(5.9f * LEFT + 2.0f * UP + 1.1f * RIGHT + 0.5f * UP + 6.5f * RIGHT + 1.2f * DOWN + 0.8f * DOWN + 0.1f * RIGHT) & LAST_MOVE & TIME(2.0f)

        // clang-format on
    });

    // 8.3cm

    inline constexpr auto PATH = Compiler::compile(COMMANDS);
    inline constexpr auto TARGET_TIMES = Compiler::getTargetTimes(COMMANDS, PATH, TARGET_TIME);
    inline constexpr auto TURN_TIMES = Compiler::TargetTime::getTurnTimes(PATH);
    inline constexpr Vec2 DESTINATION = Compiler::getDestination(PATH) / SQUARE_SIZE;
}
