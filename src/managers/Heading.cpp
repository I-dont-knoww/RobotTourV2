#include "managers/Heading.hpp"

#include "Constants.hpp"

#include "state/Radians.hpp"
#include "state/Vector.hpp"

#include <algorithm>

float Heading::update(Vec2 const& currentPosition, float dt) {
    Radians const angle = (m_targetPosition - currentPosition).angle();

    return m_reverse ? angle + Radians{ Constants::PI } : angle;
}
