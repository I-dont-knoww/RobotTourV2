#pragma once

#include "path/Compiler.hpp"
#include "path/Path.hpp"

#include "state/Vector.hpp"

#include <array>

namespace Competition {
    using namespace Compiler::Tokens;

    using Chassis::DOWEL_DISTANCE;
    using Track::SQUARE_SIZE;

    // fix fucking super speed turns and also at low speeds it lowk does the weird thing at the end
    // where it feels like the controller goes insane

    inline constexpr auto FIRST_MOVE = moveby((SQUARE_SIZE / 2.0f + DOWEL_DISTANCE) * UP) &
                                       CENTIMETERS;
    inline constexpr auto LAST_MOVE = moveby((SQUARE_SIZE - DOWEL_DISTANCE) * LEFT) & CENTIMETERS;

    inline constexpr float TARGET_TIME = 0.0f;
    inline constexpr auto COMMANDS = std::to_array<Compiler::Command>({
        // clang-format off

        // moveby(UP) & METERS,
        // moveby(3.0f * UP + LEFT) & METERS,
        // moveby(2.0f * UP) & METERS,
        // moveby(3.0f * UP + RIGHT) & METERS,
        // moveby(UP) & METERS,

        FIRST_MOVE,

        moveby(UP),
        moveby(RIGHT),
        moveby(DOWN),
        moveby(RIGHT),
        moveby(UP),
        moveby(DOWN) & REVERSE,
        moveby(LEFT),
        moveby(UP),
        moveby(LEFT),
        moveby(LEFT),
        moveby(DOWN),
        moveby(LEFT),
        moveby(RIGHT) & REVERSE,
        moveby(UP),
        moveby(UP),
        moveby(LEFT),
        moveby(RIGHT),
        moveby(UP),
        moveby(RIGHT),
        moveby(RIGHT),
        moveby(RIGHT),
        moveby(LEFT) & REVERSE,
        moveby(LEFT),

        moveby((SQUARE_SIZE - DOWEL_DISTANCE) * LEFT + 5.5f * DOWN) & CENTIMETERS

        // clang-format on
    });

    inline constexpr auto PATH = Compiler::compile(COMMANDS);
    inline constexpr auto TARGET_TIMES = Compiler::getTargetTimes(COMMANDS, PATH, TARGET_TIME);
    inline constexpr Vec2 DESTINATION = Compiler::getDestination(PATH) / SQUARE_SIZE;
}
