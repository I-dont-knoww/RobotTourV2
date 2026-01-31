#pragma once

#include "command/Command.hpp"

#include "path/Path.hpp"

namespace Path {
    template <size_t N>
    constexpr std::array<Path, N> accumulate(std::array<Command::Command, N> const& commands) {
        std::array<Path, N> path{};
        Vec2 currentPosition{ 0.0f, 0.0f };

        for (size_t i = 0; i < N; ++i) {
            Command::Command const& currentCommand = commands[i];

            if (currentCommand.relative)
                currentPosition += currentCommand.amount * currentCommand.units;
            else currentPosition = currentCommand.amount * currentCommand.units;

            Path::Flag newFlags = 0b00000000;

            if (currentCommand.flags & Command::Command::REVERSE) newFlags |= Path::REVERSE;

            if (currentCommand.flags & Command::Command::STOP) newFlags |= Path::STOP;
            else if (i == commands.size() - 1) newFlags |= Path::STOP;
            else {
                Command::Command const& nextCommand = commands[i + 1];

                if ((currentCommand.flags & Command::Command::REVERSE) !=
                    (nextCommand.flags & Command::Command::REVERSE))
                    newFlags |= Path::STOP;
                if (Vec2::dot(currentCommand.amount, nextCommand.amount) < 0.0f)
                    newFlags |= Path::STOP;
            }

            path[i] = Path{ currentPosition, newFlags };
        }

        for (size_t i = 0; i < N; ++i) {
            Command::Command const& currentCommand = commands[i];
            Path& currentPathSegment = path[i];

            currentPathSegment.position += currentCommand.offset;
        }

        return path;
    }
}
