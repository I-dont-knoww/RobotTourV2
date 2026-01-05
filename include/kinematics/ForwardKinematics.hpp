#pragma once

#include "Constants.hpp"

#include "filters/LagFilter.hpp"
#include "filters/RCFilter.hpp"

#include "state/Vector.hpp"

#include <optional>
#include <utility>

class ForwardKinematics {
public:
    ForwardKinematics(Vec2 const& wheelAngles, float dt);

    struct State {
        Vec2 position{};
        Vec2 velocity{};

        Vec2 wheelSpeeds{};

        float angle{};
        float angularVelocity{};
    };

    void update(Vec2 const& wheelAngles, std::optional<float> heading,
                std::optional<float> angularVelocity);

    constexpr decltype(auto) state(this auto&& self) {
        return std::forward_like<decltype(self)>(self.m_state);
    }

private:
    RCFilter m_velocityXFilter;
    RCFilter m_velocityYFilter;

    RCFilter m_leftWheelSpeedFilter;
    RCFilter m_rightWheelSpeedFilter;

    RCFilter m_angularVelocityFilter;

    State m_state{};

    Vec2 m_prevPosition{ 0.0f, 0.0f };
    Vec2 m_prevWheelAngles{};
    float m_prevTheta = Constants::PI / 2.0f;

    float const m_dt{};
};
