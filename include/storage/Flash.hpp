#pragma once

#include "hardware/address_mapped.h"
#include "hardware/flash.h"

#include "pico/flash.h"
#include "pico/stdlib.h"
#include "pico/sync.h"

#include <algorithm>
#include <bit>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <tuple>

namespace Flash {
    template <typename T>
        requires std::is_trivially_copyable_v<T>
    void store(T const& data) {
        size_t const dataSize = sizeof(data);
        auto const rawData = std::bit_cast<std::array<uint8_t, sizeof(data)>>(data);

        size_t const sectorSize = (dataSize + FLASH_SECTOR_SIZE - 1u) / FLASH_SECTOR_SIZE *
                                  FLASH_SECTOR_SIZE;
        size_t const pageSize = (dataSize + FLASH_PAGE_SIZE - 1u) / FLASH_PAGE_SIZE *
                                FLASH_PAGE_SIZE;
        uint32_t const offset = PICO_FLASH_SIZE_BYTES - static_cast<uint32_t>(sectorSize);

        std::array<uint8_t, pageSize> paddedData{};
        std::copy(rawData.begin(), rawData.end(), paddedData.begin());

        uint32_t interrupts = save_and_disable_interrupts();
        flash_range_erase(offset, sectorSize);
        flash_range_program(offset, paddedData.data(), pageSize);
        restore_interrupts_from_disabled(interrupts);
    }

    template <typename T>
        requires std::is_trivially_copyable_v<T>
    T load() {
        size_t const dataSize = sizeof(T);
        size_t const sectorSize = (dataSize + FLASH_SECTOR_SIZE - 1u) / FLASH_SECTOR_SIZE *
                                  FLASH_SECTOR_SIZE;
        uint32_t const offset = PICO_FLASH_SIZE_BYTES - static_cast<uint32_t>(sectorSize);

        uint8_t const* const start = reinterpret_cast<uint8_t const*>(XIP_BASE + offset);
        std::array<uint8_t, dataSize> rawData{};
        std::copy(start, start + dataSize, rawData.begin());

        return std::bit_cast<T>(rawData);
    }
}
