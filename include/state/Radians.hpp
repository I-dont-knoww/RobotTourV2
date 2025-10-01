#pragma once

#include "Constants.hpp"

#include <cmath>

class Radians {
public:
    constexpr Radians() = default;
    constexpr Radians(float value) : m_angle{ squish(value) } {}
    constexpr Radians& operator=(float value) {
        m_angle = squish(value);
        return *this;
    }

    constexpr bool equals(Radians rad, float tolerance = DEFAULT_TOLERANCE) const {
        return std::fabsf(*this - rad) <= tolerance;
    }
    constexpr friend bool operator==(Radians a, Radians b) { return a.equals(b); }
    constexpr friend bool operator!=(Radians a, Radians b) { return !a.equals(b); }

    constexpr Radians operator-() const { return Radians{ -m_angle }; }

    constexpr friend Radians operator+(Radians a, Radians b) {
        return Radians{ a.m_angle + b.m_angle };
    }
    constexpr friend Radians operator-(Radians a, Radians b) {
        return Radians{ a.m_angle - b.m_angle };
    }
    constexpr Radians& operator+=(Radians rad) {
        m_angle += rad.m_angle;
        squish();
        return *this;
    }
    constexpr Radians& operator-=(Radians rad) {
        m_angle -= rad.m_angle;
        squish();
        return *this;
    }

    constexpr friend Radians operator*(Radians rad, float n) { return Radians{ rad.m_angle * n }; }
    constexpr friend Radians operator*(float n, Radians rad) { return Radians{ rad.m_angle * n }; }
    constexpr friend Radians operator/(Radians rad, float n) { return Radians{ rad.m_angle / n }; }
    constexpr Radians& operator*=(float n) {
        m_angle *= n;
        squish();
        return *this;
    }
    constexpr Radians& operator/=(float n) {
        m_angle /= n;
        return *this;
    }

    constexpr float toFloat() const { return m_angle; }
    constexpr operator float() const { return toFloat(); }

    constexpr float toRot() const { return m_angle / (2.0f * Constants::PI); }
    constexpr float toDeg() const { return 180.0f * m_angle / Constants::PI; }

    static constexpr Radians fromRot(float rotations) {
        return Radians{ 2.0f * Constants::PI * rotations };
    }
    static constexpr Radians fromDeg(float degrees) {
        return Radians{ Constants::PI * degrees / 180.0f };
    }

    static constexpr float DEFAULT_TOLERANCE = 0.001f;

private:
    float m_angle{ 0.0f };

    constexpr void squish() { m_angle = squish(m_angle); }
    static constexpr float squish(float rad) { return std::remainderf(rad, 2.0f * Constants::PI); }
};
