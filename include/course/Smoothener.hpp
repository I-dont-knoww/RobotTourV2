#pragma once

#include "Constants.hpp"

#include "course/Course.hpp"

#include "state/Vector.hpp"

#include <array>
#include <cstddef>
#include <cstdint>

namespace Course::Smoothener {
    namespace Detail {
        constexpr auto getNewSegmentCount(size_t segmentCount, size_t routeCount) {
            using Track::MOVING_AVERAGE_WIDTH;
            return segmentCount + routeCount * (static_cast<size_t>(MOVING_AVERAGE_WIDTH) - 1);
        }
    }

    template <Course course>
    constexpr auto smoothen() {
        using Track::MOVING_AVERAGE_WIDTH;

        static constexpr size_t oldSegmentCount = course.segments.size();

        static constexpr size_t routeCount = course.routes.size();
        static constexpr size_t newSegmentCount = Detail::getNewSegmentCount(oldSegmentCount,
                                                                             routeCount);

        std::array<Segment, newSegmentCount> courseSegments{};
        std::array<Route, routeCount> courseRoutes{};
        size_t courseSegmentIndex = 0;

        for (size_t routeIndex = 0; routeIndex < routeCount; ++routeIndex) {
            Route const& oldRoute = course.routes[routeIndex];
            ptrdiff_t const currentBeginIndex = static_cast<ptrdiff_t>(oldRoute.beginIndex);
            ptrdiff_t const currentEndIndex = static_cast<ptrdiff_t>(oldRoute.endIndex);

            size_t const newBeginIndex = courseSegmentIndex;

            for (ptrdiff_t i = currentBeginIndex - MOVING_AVERAGE_WIDTH + 1; i < currentEndIndex;
                 ++i) {
                Vec2 sum{ 0.0f, 0.0f };
                float count = 0.0f;

                for (ptrdiff_t j = 0; j < MOVING_AVERAGE_WIDTH; ++j) {
                    if (i + j < currentBeginIndex || i + j >= currentEndIndex) continue;
                    sum += course.segments[static_cast<size_t>(i + j)].position;
                    count += 1.0f;
                }

                Vec2 const average = sum / count;
                courseSegments[courseSegmentIndex++] = { average, 0.0f };
            }

            size_t const newEndIndex = courseSegmentIndex;
            courseRoutes[routeIndex] = { newBeginIndex, newEndIndex, 0.0f, oldRoute.flags };
        }

        return Course{ courseSegments, courseRoutes };
    }
}
