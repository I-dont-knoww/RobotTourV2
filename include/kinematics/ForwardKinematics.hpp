#pragma once

#include "Constants.hpp"

#include "filters/LagFilter.hpp"
#include "filters/RCFilter.hpp"

#include "state/Vector.hpp"

#include <optional>
#include <utility>

class ForwardKinematics {
public:
    ForwardKinematics(Vec2 const& wheelAngles);

    struct State {
        Vec2 position{};
        Vec2 velocity{};

        Vec2 wheelSpeeds{};

        float angle{};
        float angularVelocity{};
    };

    void update(Vec2 const& wheelAngles, std::optional<float> gyroscopeHeading, float dt);

    constexpr decltype(auto) state(this auto&& self) {
        return std::forward_like<decltype(self)>(self.m_state);
    }

private:
    RCFilter m_velocityXFilter{ Kinematics::Forward::RC_FILTER_CUTOFF_FREQUENCY };
    RCFilter m_velocityYFilter{ Kinematics::Forward::RC_FILTER_CUTOFF_FREQUENCY };

    LagFilter m_leftWheelSpeedFilter{ 0.05f };
    LagFilter m_rightWheelSpeedFilter{ 0.05f };

    RCFilter m_angularVelocityFilter{ Kinematics::Forward::RC_FILTER_CUTOFF_FREQUENCY };

    State m_state{ { 0.0f, 0.0f }, { 0.0f, 0.0f }, 0.0f };

    Vec2 m_prevPosition{ 0.0f, 0.0f };
    Vec2 m_prevWheelAngles{};
    float m_prevTheta{ Constants::PI / 2 };
};
