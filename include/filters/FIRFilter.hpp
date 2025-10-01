#pragma once

#include <array>
#include <cstddef>

template <size_t N>
class FIRFilter {
public:
    FIRFilter() = default;

    float update(float input) {
        m_buffer[m_bufferIndex] = input;

        ++m_bufferIndex;
        m_bufferIndex %= N;

        float output = 0.0f;
        size_t sumIndex = m_bufferIndex;
        for (size_t i = 0; i < N; ++i) {
            if (sumIndex > 0) sumIndex--;
            else sumIndex = N - 1;

            output += m_impulseResponse[i] * m_buffer[sumIndex];
        }

        return output;
    }

private:
    std::array<float, N> m_impulseResponse{};
    std::array<float, N> m_buffer{};
    size_t m_bufferIndex{ 0u };
};
