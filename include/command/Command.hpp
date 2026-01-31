#pragma once

#include "state/Vector.hpp"

#include <cstdint>

namespace Command {
    struct Command {
        using Flag = uint8_t;
        static constexpr Flag NO_FLAGS = 0b00000000;
        static constexpr Flag STOP = 0b00000001;
        static constexpr Flag REVERSE = 0b00000010;

        Vec2 amount{};
        Vec2 offset{};
        float units{};

        Flag flags{};
        bool relative{};
    };
}
