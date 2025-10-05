#pragma once

#include "pico/sync.h"

#include <bit>
#include <concepts>
#include <cstring>
#include <utility>

template <typename T>
    requires std::is_trivially_copyable_v<T> && std::is_trivially_move_assignable_v<T> &&
             std::is_trivially_move_constructible_v<T>
class Atomic {
public:
    Atomic() { critical_section_init(&m_criticalSection); }

    Atomic(Atomic const&) = delete;
    Atomic(Atomic&&) = delete;
    Atomic& operator=(Atomic const&) = delete;
    Atomic& operator=(Atomic&&) = delete;

    T load() {
        uint8_t rawData[sizeof(T)];

        critical_section_enter_blocking(&m_criticalSection);
        for (size_t i = 0; i < sizeof(T); ++i) rawData[i] = m_rawData[i];
        critical_section_exit(&m_criticalSection);

        return std::bit_cast<T>(rawData);
    }

    void store(T const& data) {
        uint8_t rawData[sizeof(T)];
        std::memcpy(rawData, &data, sizeof(T));

        critical_section_enter_blocking(&m_criticalSection);
        for (size_t i = 0; i < sizeof(T); ++i) m_rawData[i] = rawData[i];
        critical_section_exit(&m_criticalSection);
    }

private:
    uint8_t volatile m_rawData[sizeof(T)];
    // T volatile m_rawData{};
    critical_section_t m_criticalSection{};
};
