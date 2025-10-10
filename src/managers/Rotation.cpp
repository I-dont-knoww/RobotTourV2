#include "managers/Rotation.hpp"

#include "Constants.hpp"

#include "state/Radians.hpp"
#include "state/Vector.hpp"

#include <algorithm>

float Rotation::update(Radians currentAngle, float dt) {
    return m_targetAngle;
}
