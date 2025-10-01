#pragma once

#include "state/Vector.hpp"

#include <cstdint>

using uint = unsigned int;

class LedRGB {
public:
    LedRGB(uint redPin, uint greenPin, uint bluePin);

    void setRGB(Vec3 const& color);

private:
    uint const m_redChannel{};
    uint const m_greenChannel{};
    uint const m_blueChannel{};

    uint const m_redSlice{};
    uint const m_greenSlice{};
    uint const m_blueSlice{};
};
