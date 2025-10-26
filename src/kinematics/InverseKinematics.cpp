#include "kinematics/InverseKinematics.hpp"

#include "Constants.hpp"

Vec2 InverseKinematics::update(float linearVelocity, float angularVelocity) {
    float const edgeVelocity = angularVelocity * Chassis::AXLE_LENGTH / 2.0f;
    return { linearVelocity + edgeVelocity, linearVelocity - edgeVelocity };
}
