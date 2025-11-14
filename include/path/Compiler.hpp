#pragma once

#include "Constants.hpp"

#include "state/Radians.hpp"
#include "state/Vector.hpp"

#include "path/Path.hpp"

#include <array>
#include <cstddef>
#include <numeric>
#include <optional>
#include <tuple>

using uint = unsigned int;

namespace Compiler {
    struct Command {
        Vec2 amount{};
        Vec2 offset{};
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

    struct OffsetOnce {
        Vec2 amount{};
    };
    struct OffsetAll {
        Vec2 amount{};
    };

    struct LastMove {};

    namespace Tokens {
        using Chassis::DOWEL_DISTANCE;
        using Track::SQUARE_SIZE;

        inline constexpr Vec2 UP{ 0.0f, 1.0f };
        inline constexpr Vec2 DOWN{ 0.0f, -1.0f };
        inline constexpr Vec2 LEFT{ -1.0f, 0.0f };
        inline constexpr Vec2 RIGHT{ 1.0f, 0.0f };

        inline constexpr Reverse REVERSE{};
        inline constexpr Stop STOP{};
        constexpr Time TIME(float value) { return { value }; }

        inline constexpr Centimeters CENTIMETERS{};
        inline constexpr Meters METERS{};

        constexpr OffsetOnce OFFSET_ONCE(Vec2 const& value) { return { value }; }
        constexpr OffsetAll OFFSET_ALL(Vec2 const& value) { return { value }; }

        inline constexpr LastMove LAST_MOVE{};

        constexpr Command moveby(Vec2 const& amount) {
            return { amount, { 0.0f, 0.0f }, std::nullopt, SQUARE_SIZE, Path::NO_FLAGS, true };
        }
        constexpr Command moveto(Vec2 const& amount) {
            return { amount, { 0.0f, 0.0f }, std::nullopt, SQUARE_SIZE, Path::NO_FLAGS, false };
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

        for (size_t i = 0; i < commands.size(); ++i) {
            Command const& command = commands[i];
            Path& pathSegment = path[i];

            pathSegment.position += command.offset;
        }

        return path;
    }

    template <size_t N>
    constexpr float getEffectiveLength(std::array<Command, N> const& commands,
                                       std::array<Path, N> const& path) {
        Vec2 previousPosition{ 0.0f, 0.0f };
        float totalLength = 0.0f;
        for (size_t i = 0; i < N; ++i) {
            Vec2 const& currentPosition = path[i].position;

            if (!commands[i].targetTime.has_value())
                totalLength += (currentPosition - previousPosition).length();

            previousPosition = currentPosition;
        }
        return totalLength;
    }

    template <size_t N>
    constexpr std::array<float, N> getTurnTimes(std::array<Path, N> const& path) {
        using Manager::Rotation::MAX_SPEED;
        using Manager::Rotation::TURN_TIME_OFFSET;

        std::array<float, N> turnTimes{ 0.0f };

        Vec2 firstPosition{ 0.0f, 0.0f };
        Vec2 secondPosition = path[0].position;
        for (size_t i = 1; i < N; ++i) {
            Vec2 const& thirdPosition = path[i].position;

            if (path[i - 1].flags & Path::STOP) {
                Radians angle1{ (thirdPosition - secondPosition).angle() };
                Radians angle2{ (secondPosition - firstPosition).angle() };

                if (path[i].flags & Path::REVERSE) angle1 += Constants::PI;
                if (path[i - 1].flags & Path::REVERSE) angle2 += Constants::PI;

                float angle = angle1 - angle2;
                if (angle < 0.0f) angle = -angle;

                if (angle > 1.0e-6f) turnTimes[i] = angle / MAX_SPEED + TURN_TIME_OFFSET;
            }

            firstPosition = secondPosition;
            secondPosition = thirdPosition;
        }

        return turnTimes;
    }

    template <size_t N>
    constexpr float getTotalForcedTargetTime(std::array<Command, N> const& commands,
                                             std::array<Path, N> const& path) {
        float totalForcedTargetTime = 0.0f;
        for (size_t i = 0; i < N; ++i)
            if (commands[i].targetTime.has_value())
                totalForcedTargetTime += commands[i].targetTime.value();
        return totalForcedTargetTime;
    }

    template <size_t N>
    constexpr std::array<float, N> getTargetTimes(std::array<Command, N> const& commands,
                                                  std::array<Path, N> const& path,
                                                  float targetTime) {
        float effectiveLength = getEffectiveLength(commands, path);

        std::array<float, N> turnTimes = getTurnTimes(path);
        float totalTurnTime = std::accumulate(turnTimes.begin(), turnTimes.end(), 0.0f);
        float totalForcedTargetTime = getTotalForcedTargetTime(commands, path);
        float effectiveTargetTime = targetTime - totalTurnTime - totalForcedTargetTime;

        std::array<float, N> targetTimes{};
        float accumulatedTime = 0.0f;
        Vec2 prevPosition{ 0.0f, 0.0f };
        for (size_t i = 0; i < N; ++i) {
            Vec2 const& currentPosition = path[i].position;

            accumulatedTime += commands[i].targetTime.value_or(
                effectiveTargetTime * (currentPosition - prevPosition).length() / effectiveLength +
                turnTimes[i]);
            targetTimes[i] = accumulatedTime;

            prevPosition = currentPosition;
        }
        return targetTimes;
    }

    template <size_t N>
    constexpr Vec2 getDestination(std::array<Path, N> const& path) {
        return path[N - 1].position;
    }
}
