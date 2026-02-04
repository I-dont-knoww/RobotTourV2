#pragma once

#include "Constants.hpp"

#include "course/Course.hpp"

#include "state/Vector.hpp"

#include <array>
#include <cstddef>

namespace Course::Extender {
    namespace Detail {
        constexpr size_t ceil(float x) {
            size_t const casted = static_cast<size_t>(x);
            return x == static_cast<float>(casted) ? casted : casted + 1;
        }

        template <size_t segmentCount, size_t routeCount>
        constexpr size_t getNewSegmentCount(Course<segmentCount, routeCount> const& course) {
            using Track::PATH_SPACING;

            size_t extendedSegmentCount = 0;

            for (size_t i = 0; i < course.segments.size() - 1; ++i) {
                Segment const& currentSegment = course.segments[i];
                Segment const& nextSegment = course.segments[i + 1];

                Vec2 const pathVector = nextSegment.position - currentSegment.position;
                extendedSegmentCount += ceil(pathVector.length() / PATH_SPACING);
            }

            return extendedSegmentCount + routeCount;
        }
    }

    template <Course course>
    constexpr auto extend() {
        using Track::PATH_SPACING;

        static constexpr size_t extendedSegmentCount = Detail::getNewSegmentCount(course);
        static constexpr size_t routeCount = course.routes.size();

        std::array<Segment, extendedSegmentCount> courseSegments{};
        std::array<Route, routeCount> courseRoutes{};
        size_t courseSegmentIndex = 0;

        for (size_t routeIndex = 0; routeIndex < routeCount; ++routeIndex) {
            Route const& oldRoute = course.routes[routeIndex];
            size_t const currentBeginIndex = oldRoute.beginIndex;
            size_t const currentEndIndex = oldRoute.endIndex;

            size_t const newBeginIndex = courseSegmentIndex;

            for (size_t i = currentBeginIndex; i < currentEndIndex - 1; ++i) {
                Segment const& currentSegment = course.segments[i];
                Segment const& nextSegment = course.segments[i + 1];

                Vec2 const pathVector = nextSegment.position - currentSegment.position;
                size_t segmentsToAdd = Detail::ceil(pathVector.length() / PATH_SPACING);
                Vec2 const newSegmentVector = pathVector / pathVector.length() * PATH_SPACING;

                for (size_t j = 0; j < segmentsToAdd; ++j) {
                    Vec2 const newPosition = currentSegment.position +
                                             static_cast<float>(j) * newSegmentVector;
                    courseSegments[courseSegmentIndex++].position = newPosition;
                }
            }
            Vec2 const lastSegmentPosition = course.segments[currentEndIndex - 1].position;
            courseSegments[courseSegmentIndex++].position = lastSegmentPosition;

            size_t const newEndIndex = courseSegmentIndex;
            courseRoutes[routeIndex] = { newBeginIndex, newEndIndex, 0.0f, oldRoute.flags };
        }

        return Course{ courseSegments, courseRoutes };
    }
}
