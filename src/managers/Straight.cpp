#include "managers/Straight.hpp"
#include "Constants.hpp"

#include "course/Course.hpp"

#include "state/Radians.hpp"
#include "state/Vector.hpp"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <optional>
#include <utility>

Straight::Straight(float dt) : m_dt{ dt } {}

static float getLookaheadDistance(Vec2 const& currentVelocity, Radians currentAngle) {
    using Manager::Straight::LOOK_AHEAD_KV;
    using Manager::Straight::MAX_LOOK_AHEAD;
    using Manager::Straight::MIN_LOOK_AHEAD;

    float const angleDifference = Radians{ currentVelocity.angle() + Constants::PI / 2.0f } -
                                  currentAngle;
    float const currentLinearVelocity = std::copysignf(currentVelocity.length(), angleDifference);

    return std::clamp(LOOK_AHEAD_KV * currentLinearVelocity, MIN_LOOK_AHEAD, MAX_LOOK_AHEAD);
}

static bool pointCircleIntersection(Vec2 const& point, Vec2 const& circleCenter,
                                    float circleRadius) {
    return (point - circleCenter).lengthSquared() <= circleRadius * circleRadius;
}

static std::optional<Vec2> lineCircleIntersectionProjections(Vec2 const& lineStart,
                                                             Vec2 const& lineEnd,
                                                             Vec2 const& circleCenter,
                                                             float circleRadius) {
    Vec2 const lineVector = lineEnd - lineStart;
    Vec2 const circleToLineVector = lineStart - circleCenter;

    float const a = Vec2::dot(lineVector, lineVector);
    float const b = 2.0f * Vec2::dot(circleToLineVector, lineVector);
    float const c = Vec2::dot(circleToLineVector, circleToLineVector) - circleRadius * circleRadius;

    float const discriminant = b * b - 4.0f * a * c;
    if (discriminant < 0.0f) return std::nullopt;
    else {
        float const sqrtDiscriminant = std::sqrtf(discriminant);
        float const t1 = (-b - sqrtDiscriminant) / (2.0f * a);
        float const t2 = (-b + sqrtDiscriminant) / (2.0f * a);

        return Vec2{ t1, t2 };
    }
}

static std::optional<Vec2> lineCircleIntersection(Vec2 const& lineStart, Vec2 const& lineEnd,
                                                  Vec2 const& circleCenter, float circleRadius) {
    auto const projections = lineCircleIntersectionProjections(lineStart, lineEnd, circleCenter,
                                                               circleRadius);
    if (!projections) return std::nullopt;

    Vec2 const lineVector = lineEnd - lineStart;

    bool intersection1 = projections->x >= 0.0f && projections->x <= 1.0f;
    bool intersection2 = projections->y >= 0.0f && projections->y <= 1.0f;

    if (intersection1 && intersection2) {
        if (projections->x >= projections->y) return lineStart + projections->x * lineVector;
        else return lineStart + projections->y * lineVector;
    }

    if (intersection1) return lineStart + projections->x * lineVector;
    if (intersection2) return lineStart + projections->y * lineVector;
    return std::nullopt;
}

Vec2 Straight::getNextGoalPoint(Vec2 const& currentPosition, float lookAheadDistance) {
    if (m_previousGoalPointIterator == std::end(m_route) - 1) {
        Vec2 const& finalStartLine = (std::end(m_route) - 2)->position;
        Vec2 const& finalEndLine = (std::end(m_route) - 1)->position;

        auto const projections = lineCircleIntersectionProjections(
            finalStartLine, finalEndLine, currentPosition, lookAheadDistance);

        if (projections) {
            Vec2 const& finalLineVector = finalEndLine - finalStartLine;
            return finalStartLine + std::max(projections->x, projections->y) * finalLineVector;
        } else return (std::end(m_route) - 1)->position;
    }

    for (auto routeIterator = m_previousGoalPointIterator; routeIterator != std::end(m_route);
         ++routeIterator) {
        Vec2 const& previousLinePosition = (routeIterator - 1)->position;
        Vec2 const& currentLinePosition = routeIterator->position;

        bool const canSeeEndPoint = pointCircleIntersection(currentLinePosition, currentPosition,
                                                            lookAheadDistance);
        if (canSeeEndPoint) {
            m_previousGoalPointIterator++;
            break;
        }

        std::optional<Vec2> const intersection = lineCircleIntersection(
            previousLinePosition, currentLinePosition, currentPosition, lookAheadDistance);

        if (intersection) {
            m_previousGoalPoint = *intersection;
            m_previousGoalPointIterator = routeIterator;

            return *intersection;
        }
    }

    return m_previousGoalPoint;
}

Straight::RouteIterator Straight::getNextUnpassedSegment(Vec2 const& currentPosition) {
    for (auto routeIterator = m_previousPassedSegmentIterator; routeIterator != std::end(m_route);
         ++routeIterator) {
        Vec2 const& previousPathPosition = (routeIterator - 1)->position;
        Vec2 const& currentPathPosition = routeIterator->position;

        Vec2 const lineVector = currentPathPosition - previousPathPosition;
        bool const isPassedSegment = Vec2::dot(currentPosition - previousPathPosition, lineVector) >
                                     Vec2::dot(lineVector, lineVector);

        if (!isPassedSegment) return m_previousPassedSegmentIterator = routeIterator;
    }

    return m_previousPassedSegmentIterator = std::end(m_route);
}

static float getDistanceAlongLine(Vec2 const& lineStart, Vec2 const& lineEnd, Vec2 const& point) {
    Vec2 const lineVector = lineEnd - lineStart;
    Vec2 const pointVector = point - lineStart;

    float const t = Vec2::dot(pointVector, lineVector) / Vec2::dot(lineVector, lineVector);
    return (t * lineVector).length();
}

static float getMaximumLinearVelocity(Course::Segment const& passedSegment,
                                      Course::Segment const& unpassedSegment,
                                      float distanceAlongCurrentSegment) {
    using Track::MAX_ACCELERATION;

    float const segmentLength = (unpassedSegment.position - passedSegment.position).length();
    float const distanceToUnpassedSegment = segmentLength - distanceAlongCurrentSegment;

    float const initialSpeed = passedSegment.velocityLimit;
    float const finalSpeed = unpassedSegment.velocityLimit;

    if (finalSpeed >= initialSpeed) return finalSpeed;
    else {
        float const maxSpeedSquared = finalSpeed * finalSpeed +
                                      2.0f * MAX_ACCELERATION * distanceToUnpassedSegment;
        return std::sqrtf(maxSpeedSquared);
    }
}

static float getDistanceLeft(Vec2 const& passedPosition, Course::Segment const& unpassedSegment,
                             float distanceAlongCurrentSegment) {
    float const distanceToEnd = unpassedSegment.distanceToEnd;
    float const lengthOfCurrentSegment = (unpassedSegment.position - passedPosition).length();
    return distanceToEnd + -distanceAlongCurrentSegment;
}

static float getTargetLinearVelocity(float distanceLeft, float timeLeft) {
    using Track::MAX_ACCELERATION;
    using Track::MAX_VELOCITY;
    using Track::MIN_VELOCITY;

    if (timeLeft <= 0.0f) return MAX_VELOCITY;
    if (distanceLeft / timeLeft <= MIN_VELOCITY) return distanceLeft / timeLeft;

    float const determinant = MAX_ACCELERATION * MAX_ACCELERATION * timeLeft * timeLeft +
                              2.0f * MAX_ACCELERATION * (MIN_VELOCITY * timeLeft - distanceLeft);
    if (determinant <= 0.0f) return MAX_VELOCITY;
    else return MIN_VELOCITY + MAX_ACCELERATION * timeLeft - std::sqrtf(determinant);
}

float Straight::getLimitedLinearVelocity(Vec2 const& currentPosition,
                                         RouteIterator unpassedSegmentIterator, float timeLeft) {
    Course::Segment const& passedSegment = *(unpassedSegmentIterator - 1);
    Course::Segment const& unpassedSegment = *(unpassedSegmentIterator);

    float const distanceAlongCurrentSegment = getDistanceAlongLine(
        passedSegment.position, unpassedSegment.position, currentPosition);
    float const maximumLinearVelocity = getMaximumLinearVelocity(passedSegment, unpassedSegment,
                                                                 distanceAlongCurrentSegment);
    float const distanceLeftAlongSegment = getDistanceLeft(passedSegment.position, unpassedSegment,
                                                           distanceAlongCurrentSegment);
    float const targetLinearVelocity = getTargetLinearVelocity(distanceLeftAlongSegment, timeLeft);
    float limitedLinearVelocity = std::min(targetLinearVelocity, maximumLinearVelocity);

    float const maxVelocityChange = std::fabsf(limitedLinearVelocity - m_previousVelocity);
    return m_previousVelocity += std::clamp(limitedLinearVelocity - m_previousVelocity,
                                            -maxVelocityChange, maxVelocityChange);
}

static float getTargetAngularVelocity(Vec2 const& currentPosition, Vec2 const& goalPoint,
                                      Radians currentAngle, float targetLinearVelocity,
                                      float lookaheadDistance) {
    using Chassis::AXLE_LENGTH;

    Radians const angleToGoal = (goalPoint - currentPosition).angle();
    Radians const angularError = angleToGoal - currentAngle;

    return AXLE_LENGTH * std::sinf(angularError) * targetLinearVelocity / lookaheadDistance;
}

void Straight::setup(Route route, float targetTime) {
    m_route = route;
    m_previousGoalPointIterator = std::begin(route) + 1;
    m_previousPassedSegmentIterator = std::begin(route) + 1;

    m_previousVelocity = 0.0f;
    m_targetTime = targetTime;
}

std::optional<Vec2> Straight::update(Vec2 const& currentPosition, Vec2 const& currentVelocity,
                                     Radians currentAngle, float currentTime) {
    RouteIterator const unpassedSegmentIterator = getNextUnpassedSegment(currentPosition);
    if (unpassedSegmentIterator == std::end(m_route)) return std::nullopt;

    float const timeLeft = m_targetTime - currentTime;
    float const targetLinearVelocity = getLimitedLinearVelocity(currentPosition,
                                                                unpassedSegmentIterator, timeLeft);

    float const lookaheadDistance = getLookaheadDistance(currentVelocity, currentAngle);
    std::printf(">lookahead:%.5f\n", lookaheadDistance);
    Vec2 const goalPoint = getNextGoalPoint(currentPosition, lookaheadDistance);

    float const targetAngularVelocity = getTargetAngularVelocity(
        currentPosition, goalPoint, currentAngle, targetLinearVelocity, lookaheadDistance);

    return Vec2{ targetLinearVelocity, targetAngularVelocity };
}
