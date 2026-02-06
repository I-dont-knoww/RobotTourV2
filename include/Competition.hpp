#pragma once

#include "command/Command.hpp"
#include "command/Tokens.hpp"

#include "course/Compiler.hpp"

#include "path/Accumulate.hpp"
#include "path/Path.hpp"

#include "state/Vector.hpp"

#include <array>

// are you on the right branch?
// did you set the FAIL_RUN variable?
// did you set the target time?
// did you check the destination?
// is the path rotated the right way in your mind?

namespace Competition {
    using namespace Command::Tokens;

    inline constexpr bool FAIL_RUN = false;
    inline constexpr float TARGET_TIME = 0.0f;

    inline constexpr auto COMMANDS = std::to_array<Command::Command>({
        // clang-format off

        // moveby(UP),
        // moveby(0.6f * LEFT)
        
        moveby(1.2f * UP),
        moveby(1.2f * UP + 1.0f * LEFT),
        moveby(1.2f * UP),
        moveby(1.2f * UP + 1.0f * RIGHT),
        moveby(1.2f * UP),

        // clang-format on
    });

    inline constexpr auto PATH = Path::accumulate(COMMANDS);
    inline constexpr auto COMPILED = Course::compile<PATH, TARGET_TIME>();
    inline constexpr auto SEGMENTS = COMPILED.segments;
    inline constexpr auto ROUTES = COMPILED.routes;
}
