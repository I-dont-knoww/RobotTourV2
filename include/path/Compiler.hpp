#pragma once

#include "Constants.hpp"

#include "state/Vector.hpp"

#include "path/Path.hpp"

#include <array>
#include <cstddef>
#include <optional>

using uint = unsigned int;

namespace Compiler {
    struct Command {
        Vec2 amount{};
        std::optional<float> targetTime{};
        float units{};

        uint flags{};
        bool relative{};
    };

    struct Reverse {};
    struct Stop {};
    struct Time {
        float value{};
    };

    struct Centimeters {};
    struct Meters {};

    namespace Tokens {
        inline constexpr Vec2 UP{ 0.0f, 1.0f };
        inline constexpr Vec2 DOWN{ 0.0f, -1.0f };
        inline constexpr Vec2 LEFT{ -1.0f, 0.0f };
        inline constexpr Vec2 RIGHT{ 1.0f, 0.0f };

        inline constexpr Reverse REVERSE{};
        inline constexpr Stop STOP{};
        constexpr Time TIME(float value) { return Time{ value }; }

        inline constexpr Centimeters CENTIMETERS{};
        inline constexpr Meters METERS{};

        constexpr Command moveby(Vec2 const& amount) {
            return { amount, std::nullopt, Track::SQUARE_SIZE, Path::NO_FLAGS, true };
        }
        constexpr Command moveto(Vec2 const& amount) {
            return { amount, std::nullopt, Track::SQUARE_SIZE, Path::NO_FLAGS, false };
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
        constexpr Command operator&(Command const& command, Meters) {
            Command newCommand = command;
            newCommand.units = 100.0f;
            return newCommand;
        }

        constexpr Command operator&(Command const& command, Time time) {
            Command newCommand = command;
            newCommand.targetTime = time.value;
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
            } else if (Vec2::dot(command.amount, nextCommand.amount) < 0.0f ||
                       command.flags & Path::REVERSE != nextCommand.flags & Path::REVERSE)
                segment.flags |= Path::STOP;

            path[i] = segment;
        }

        return path;
    }

    template <size_t N>
    constexpr std::array<float, N> getTargetTimes(std::array<Command, N> commands,
                                                  std::array<Path, N> const& path,
                                                  float targetTime) {
        float totalLength = 0.0f;
        float adjustedTargetTime = targetTime;
        Vec2 prevPosition{ 0.0f, 0.0f };
        for (size_t i = 0; i < N; ++i) {
            Vec2 const& currentPosition = path[i].position;

            if (commands[i].targetTime.has_value())
                adjustedTargetTime -= commands[i].targetTime.value();
            else totalLength += (currentPosition - prevPosition).length();

            prevPosition = currentPosition;
        }
        assert(adjustedTargetTime < 0.0f);

        std::array<float, N> targetTimes{};
        float accumulatedTime = 0.0f;
        prevPosition = { 0.0f, 0.0f };
        for (size_t i = 0; i < N; ++i) {
            Vec2 const& currentPosition = path[i].position;

            if (commands[i].targetTime.has_value()) targetTimes[i] = commands[i].targetTime.value();
            else {
                accumulatedTime += commands[i].targetTime.value_or(
                    adjustedTargetTime * (currentPosition - prevPosition).length() / totalLength);
                targetTimes[i] = accumulatedTime;
            }

            prevPosition = currentPosition;
        }
        return targetTimes;
    }

    template <size_t N>
    constexpr Vec2 getDestination(std::array<Path, N> const& path) {
        return path[N - 1].position;
    }
}
