#pragma once

#include "Constants.hpp"

#include "state/Vector.hpp"

#include "path/Path.hpp"

#include <array>
#include <cstddef>

using uint = unsigned int;

namespace Compiler {
    struct Command {
        Vec2 amount{};
        float units{};
        uint flags{};
        bool relative{};
    };

    struct Reverse {};
    struct Stop {};

    struct Centimeters {};

    namespace Tokens {
        inline constexpr Vec2 UP{ 0.0f, 1.0f };
        inline constexpr Vec2 DOWN{ 0.0f, -1.0f };
        inline constexpr Vec2 LEFT{ -1.0f, 0.0f };
        inline constexpr Vec2 RIGHT{ 1.0f, 0.0f };

        inline constexpr Reverse REVERSE{};
        inline constexpr Stop STOP{};

        inline constexpr Centimeters CENTIMETERS{};

        constexpr Command moveby(Vec2 const& amount) {
            return { amount, Track::SQUARE_SIZE, Path::NO_FLAGS, true };
        }
        constexpr Command moveto(Vec2 const& amount) {
            return { amount, Track::SQUARE_SIZE, Path::NO_FLAGS, false };
        }

        constexpr Command operator&(Command const& command, Reverse) {
            Command newCommand = command;
            newCommand.flags |= Path::REVERSE;
            return newCommand;
        }
        constexpr Command operator&(Command const& command, Stop) {
            Command newCommand = command;
            newCommand.flags |= Path::STOP;
            return newCommand;
        }

        constexpr Command operator&(Command const& command, Centimeters) {
            Command newCommand = command;
            newCommand.units = 1.0f;
            return newCommand;
        }
    }

    template <size_t N>
    constexpr std::array<Path, N> compile(std::array<Command, N> const& commands) {
        std::array<Path, N> path{};
        Vec2 currentPosition{ 0.0f, 0.0f };

        for (size_t i = 0; i < commands.size(); ++i) {
            Command const& command = commands[i];
            Command const& nextCommand = commands[i + 1];

            if (command.relative) currentPosition += command.amount * command.units;
            else currentPosition = command.amount * command.units;

            Path segment{ currentPosition, command.flags };

            if (i == commands.size() - 1) {
                segment.flags |= Path::STOP;
                segment.flags |= Path::ACCURATE;
            } else {
                if (Vec2::dot(command.amount, nextCommand.amount) < 0.0f)
                    segment.flags |= Path::STOP;
            }
            path[i] = segment;
        }

        return path;
    }

    template <size_t N>
    constexpr std::array<float, N> getTargetTimes(std::array<Path, N> const& path,
                                                  float targetTime) {
        float totalLength = path[0].position.length();
        for (size_t i = 0; i < N - 1; ++i) {
            Path const& segment = path[i];
            Path const& nextSegment = path[i + 1];

            totalLength += (nextSegment.position - segment.position).length();
        }

        std::array<float, N> targetTimes{ targetTime * path[0].position.length() / totalLength };
        for (size_t i = 1; i < N; ++i) {
            Path const& previousSegment = path[i - 1];
            Path const& segment = path[i];

            targetTimes[i] = targetTimes[i - 1] +
                             targetTime * (segment.position - previousSegment.position).length() /
                                 totalLength;
        }
        return targetTimes;
    }

    template <size_t N>
    constexpr Vec2 getDestination(std::array<Path, N> const& path) {
        return path[N - 1].position / Track::SQUARE_SIZE;
    }
}
