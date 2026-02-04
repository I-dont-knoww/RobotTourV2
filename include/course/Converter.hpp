#pragma once

#include "course/Course.hpp"

#include "path/Path.hpp"

#include <array>
#include <cstddef>

namespace Course::Converter {
    namespace Detail {
        template <size_t N>
        constexpr size_t getRouteCount(std::array<Path::Path, N> const& path) {
            size_t routeCount = 0;
            for (Path::Path const& pathSegment : path)
                if (pathSegment.flags & Path::Path::STOP) routeCount++;
            return routeCount;
        }

        constexpr size_t getSegmentCount(size_t pathSize, size_t routeCount) {
            return routeCount + pathSize;
        }
    }

    template <std::array path>
    constexpr auto convert() {
        static constexpr size_t routeCount = Detail::getRouteCount(path);
        static constexpr size_t segmentCount = Detail::getSegmentCount(path.size(), routeCount);

        std::array<Segment, segmentCount> courseSegments{};
        size_t courseSegmentIndex = 0;

        std::array<Route, routeCount> courseRoutes{};
        size_t courseRouteIndex = 0;
        size_t beginIndex = 0;

        courseSegments[courseSegmentIndex++] = { { 0.0f, 0.0f }, 0.0f };
        for (Path::Path const& pathSegment : path) {
            courseSegments[courseSegmentIndex++].position = pathSegment.position;

            if (pathSegment.flags & Path::Path::STOP) {
                size_t endIndex = courseSegmentIndex;
                Route::Flag routeFlags =
                    (pathSegment.flags & Path::Path::REVERSE) ? Route::REVERSE : Route::NO_FLAGS;
                courseRoutes[courseRouteIndex++] = { beginIndex, endIndex, 0.0f, routeFlags };

                beginIndex = endIndex;

                if (courseSegmentIndex < courseSegments.size())
                    courseSegments[courseSegmentIndex++].position = pathSegment.position;
            }
        }

        return Course{ courseSegments, courseRoutes };
    }
}
