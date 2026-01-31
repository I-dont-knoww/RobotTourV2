#pragma once

#include "Constants.hpp"

#include "state/Radians.hpp"
#include "state/Vector.hpp"

#include "command/Command.hpp"

#include <array>
#include <cstddef>
#include <numeric>
#include <tuple>

using uint = unsigned int;

namespace Command {
    struct Reverse {};
    struct Stop {};

    struct Centimeters {};
    struct Meters {};

    struct OffsetOnce {
        Vec2 amount{};
    };
    struct OffsetAll {
        Vec2 amount{};
    };

    struct LastMove {};
}

namespace Command::Tokens {
    using Chassis::DOWEL_DISTANCE;
    using Track::SQUARE_SIZE;

    inline constexpr Vec2 UP{ 0.0f, 1.0f };
    inline constexpr Vec2 DOWN{ 0.0f, -1.0f };
    inline constexpr Vec2 LEFT{ -1.0f, 0.0f };
    inline constexpr Vec2 RIGHT{ 1.0f, 0.0f };

    inline constexpr Reverse REVERSE{};
    inline constexpr Stop STOP{};

    inline constexpr Centimeters CENTIMETERS{};
    inline constexpr Meters METERS{};

    constexpr OffsetOnce OFFSET_ONCE(Vec2 const& value) { return { value }; }
    constexpr OffsetAll OFFSET_ALL(Vec2 const& value) { return { value }; }

    inline constexpr LastMove LAST_MOVE{};

    constexpr Command moveby(Vec2 const& amount) {
        return { amount, { 0.0f, 0.0f }, SQUARE_SIZE, Command::NO_FLAGS, true };
    }
    constexpr Command moveto(Vec2 const& amount) {
        return { amount, { 0.0f, 0.0f }, SQUARE_SIZE, Command::NO_FLAGS, false };
    }

    constexpr Command operator&(Command const& command, Reverse) {
        Command newCommand = command;
        newCommand.flags |= Command::REVERSE;
        return newCommand;
    }
    constexpr Command operator&(Command const& command, Stop) {
        Command newCommand = command;
        newCommand.flags |= Command::STOP;
        return newCommand;
    }

    constexpr Command operator&(Command const& command, Centimeters) {
        Command newCommand = command;
        newCommand.units = 1.0f;
        return newCommand;
    }
    constexpr Command operator&(Command const& command, Meters) {
        Command newCommand = command;
        newCommand.units = 100.0f;
        return newCommand;
    }

    constexpr Command operator&(Command const& command, OffsetOnce const& offset) {
        Command newCommand = command;
        newCommand.offset += offset.amount;
        return newCommand;
    }
    constexpr Command operator&(Command const& command, OffsetAll const& offset) {
        Command newCommand = command;
        newCommand.amount += offset.amount;
        return newCommand;
    }

    inline constexpr Command FIRST_MOVE = moveby((SQUARE_SIZE / 2.0f + DOWEL_DISTANCE) * UP) &
                                          CENTIMETERS;
    constexpr Command operator&(Command const& command, LastMove) {
        Command newCommand = command;

        float length = command.amount.length() * newCommand.units;
        float const scale = (length - DOWEL_DISTANCE) / length;

        newCommand.amount *= scale;
        return newCommand;
    }
}
