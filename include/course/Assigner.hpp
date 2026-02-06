#pragma once

#include "course/Course.hpp"

#include "state/Vector.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>

namespace Course::Assigner {
    namespace Detail {
        constexpr float constexprSqrt(float x) { return VectorDetail::constexprSqrt(x); }
        constexpr float constexprAbs(float x) { return x < 0.0f ? -x : x; }

        template <size_t segmentCount, size_t routeCount>
        constexpr auto getCurvatures(Course<segmentCount, routeCount> const& course) {
            std::array<float, segmentCount> curvatures{};

            for (Route const& route : course.routes) {
                size_t beginIndex = route.beginIndex;
                size_t endIndex = route.endIndex;

                curvatures[beginIndex] = 0.0f;
                curvatures[endIndex - 1] = 0.0f;

                for (size_t i = beginIndex + 1; i < endIndex - 1; ++i) {
                    Vec2 const A = course.segments[i - 1].position;
                    Vec2 const B = course.segments[i].position;
                    Vec2 const C = course.segments[i + 1].position;

                    Vec2 const AB = A - B;
                    Vec2 const BC = B - C;
                    Vec2 const AC = C - A;

                    curvatures[i] = constexprAbs(2.0f * Vec2::cross(AB, BC) /
                                                 (AB.length() * BC.length() * AC.length()));
                }
            }

            return curvatures;
        }

        template <Course course>
        constexpr auto limitCentripetal() {
            using Track::CURVATURE_SLOWDOWN;
            using Track::MAX_VELOCITY;

            static constexpr size_t segmentCount = course.segments.size();
            static constexpr std::array<float, segmentCount> curvatures = getCurvatures(course);

            Course newCourse = course;

            for (size_t i = 0; i < newCourse.segments.size(); ++i) {
                Segment& segment = newCourse.segments[i];
                float const curvature = curvatures[i];
                float const sqrtCurvature = constexprSqrt(curvature);

                if (curvature == 0.0f) segment.velocityLimit = MAX_VELOCITY;
                else
                    segment.velocityLimit = std::min(CURVATURE_SLOWDOWN / sqrtCurvature,
                                                     MAX_VELOCITY);
            }

            return newCourse;
        }

        template <Course course>
        constexpr auto limitAcceleration() {
            using Track::MAX_ACCELERATION;
            using Track::MIN_VELOCITY;

            Course newCourse = course;

            for (Route const& route : newCourse.routes) {
                ptrdiff_t beginIndex = static_cast<ptrdiff_t>(route.beginIndex);
                ptrdiff_t endIndex = static_cast<ptrdiff_t>(route.endIndex);

                newCourse.segments[static_cast<size_t>(endIndex - 1)].velocityLimit = MIN_VELOCITY;

                for (ptrdiff_t i = endIndex - 2; i >= beginIndex; --i) {
                    Segment& currentSegment = newCourse.segments[static_cast<size_t>(i)];
                    Segment const& nextSegment = newCourse.segments[static_cast<size_t>(i + 1)];
                    float const distance =
                        (nextSegment.position - currentSegment.position).length();
                    float const limitedVelocity =
                        constexprSqrt(nextSegment.velocityLimit * nextSegment.velocityLimit +
                                      2.0f * MAX_ACCELERATION * distance);

                    currentSegment.velocityLimit = std::min(limitedVelocity,
                                                            currentSegment.velocityLimit);
                }
            }

            return newCourse;
        }
    }

    template <Course course>
    constexpr auto assignVelocityLimits() {
        return Detail::limitAcceleration<Detail::limitCentripetal<course>()>();
    }

    template <size_t segmentCount, size_t routeCount>
    constexpr auto assignDistanceLeft(Course<segmentCount, routeCount> course) {
        for (Route const& route : course.routes) {
            ptrdiff_t const beginIndex = static_cast<ptrdiff_t>(route.beginIndex);
            ptrdiff_t const endIndex = static_cast<ptrdiff_t>(route.endIndex);

            course.segments[static_cast<size_t>(endIndex - 1)].distanceToEnd = 0.0f;
            for (ptrdiff_t i = endIndex - 2; i >= beginIndex; --i) {
                Segment& currentSegment = course.segments[static_cast<size_t>(i)];
                Segment const& nextSegment = course.segments[static_cast<size_t>(i + 1)];
                float const distance = (nextSegment.position - currentSegment.position).length();

                currentSegment.distanceToEnd = distance + nextSegment.distanceToEnd;
            }
        }

        return course;
    }

    template <size_t segmentCount, size_t routeCount>
    constexpr auto assignTargetTimes(Course<segmentCount, routeCount> course, float targetTime) {
        using Track::STOP_TIME;

        float totalLength = 0;
        for (Route const& route : course.routes)
            totalLength += course.segments[route.beginIndex].distanceToEnd;

        float const assignableTargetTime = std::max(0.0f, targetTime - routeCount * STOP_TIME);
        for (Route& route : course.routes) {
            float const routeLength = course.segments[route.beginIndex].distanceToEnd;
            route.targetTime = assignableTargetTime * routeLength / totalLength + STOP_TIME;
        }

        return course;
    }
}
