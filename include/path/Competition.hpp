#pragma once

#include "path/Compiler.hpp"
#include "path/Path.hpp"

#include "state/Vector.hpp"

#include <array>

namespace Competition {
    using namespace Compiler::Tokens;

    // fix fucking super speed turns and also at low speeds it lowk does the weird thing at the end
    // where it feels like the controller goes insane

    inline constexpr float TARGET_TIME = 55.0f;
    inline constexpr auto COMMANDS = std::to_array<Compiler::Command>({
        // clang-format off

        // moveby(UP),
        // moveby(RIGHT),
        // moveby(DOWN),
        // moveby(LEFT),
        // moveby(UP),
        // moveby(RIGHT),
        // moveby(DOWN),
        // moveby(LEFT),

        moveby(RIGHT),
        moveby(UP),
        moveby(UP),
        moveby(UP),
        moveby(DOWN),
        moveby(LEFT),
        moveby(LEFT),
        moveby(UP),
        moveby(LEFT),
        moveby(RIGHT),
        moveby(DOWN),
        moveby(LEFT),
        moveby(DOWN),
        moveby(DOWN),
        moveby(RIGHT),
        moveby(DOWN),
        moveby(LEFT),
        moveby(RIGHT),
        moveby(UP),
        moveby(UP),
        moveby(LEFT),
        moveby(UP),
        moveby(RIGHT),
        moveby(RIGHT),
        moveby(RIGHT),
        moveby(DOWN),
        moveby(DOWN),
        moveby(LEFT),
        moveby(UP),
        moveby(DOWN),
        moveby(RIGHT),
        moveby(UP),
        moveby(UP),
        moveby(LEFT),
        moveby(UP),

        // clang-format on
    });

    inline constexpr auto PATH = Compiler::compile(COMMANDS);
    inline constexpr auto TARGET_TIMES = Compiler::getTargetTimes(COMMANDS, PATH, TARGET_TIME);
    inline constexpr Vec2 DESTINATION = Compiler::getDestination(PATH);
}
