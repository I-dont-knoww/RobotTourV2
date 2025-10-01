#pragma once

#include "state/Vector.hpp"

#include "hardware/pio.h"

class Encoders {
public:
    Encoders(PIO pio, uint csLeftPin, uint csRightPin, uint sckPin, uint misoPin);

    Encoders(Encoders const&) = delete;
    Encoders& operator=(Encoders const&) = delete;
    Encoders(Encoders&&) = delete;
    Encoders& operator=(Encoders&&) = delete;

    Vec2 data() const;

private:
    uint32_t volatile m_rawData[2]{ 0u, 0u };
};
