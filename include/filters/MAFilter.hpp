#pragma once

#include <array>
#include <cstddef>

template <size_t FilterLength>
class MAFilter {
public:
    MAFilter() = default;

    float update(float input) {
        output -= buffer[bufferIndex];
        buffer[bufferIndex] = input;
        output += buffer[bufferIndex];

        ++bufferIndex;
        bufferIndex %= FilterLength;

        return output / static_cast<float>(FilterLength);
    }

private:
    std::array<float, FilterLength> buffer{};
    size_t bufferIndex{ 0u };

    float output{ 0.0f };
};
