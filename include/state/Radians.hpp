// https://www.elbeno.com/blog/?p=1211

#pragma once

#include "Constants.hpp"

#include <cmath>
#include <limits>

namespace RadianHelper {
    constexpr float ipow(float x, int n) {
        if (n == 0) return 1.0f;
        else if (n == 1) return x;
        else if (n > 1) return ((n & 1) ? x * ipow(x, n - 1) : ipow(x, n / 2) * ipow(x, n / 2));
        else return 1.0f / ipow(x, -n);
    }

    constexpr float floorHelper2(float x, float guess, float inc) {
        if (guess + inc <= x) return floorHelper2(x, guess + inc, inc);
        else if (inc <= 1.0f) return guess;
        else return floorHelper2(x, guess, inc / 2.0f);
    }

    constexpr float floorHelper1(float x, float guess, float inc) {
        if (inc < 8.0f) return floorHelper2(x, guess, inc);
        else if (guess + inc <= x) return floorHelper1(x, guess + inc, inc);
        else if (guess + (inc / 8.0f) * 7.0f <= x)
            return floorHelper1(x, guess + (inc / 8.0f) * 7.0f, inc / 8.0f);
        else if (guess + (inc / 8.0f) * 6.0f <= x)
            return floorHelper1(x, guess + (inc / 8.0f) * 6.0f, inc / 8.0f);
        else if (guess + (inc / 8.0f) * 5.0f <= x)
            return floorHelper1(x, guess + (inc / 8.0f) * 5.0f, inc / 8.0f);
        else if (guess + (inc / 8.0f) * 4.0f <= x)
            return floorHelper1(x, guess + (inc / 8.0f) * 4.0f, inc / 8.0f);
        else if (guess + (inc / 8.0f) * 3.0f <= x)
            return floorHelper1(x, guess + (inc / 8.0f) * 3.0f, inc / 8.0f);
        else if (guess + (inc / 8.0f) * 2.0f <= x)
            return floorHelper1(x, guess + (inc / 8.0f) * 2.0f, inc / 8.0f);
        else if (guess + inc / 8.0f <= x) return floorHelper1(x, guess + inc / 8.0f, inc / 8.0f);
        else return floorHelper1(x, guess, inc / 8.0f);
    }

    constexpr float floor(float x) {
        if (x < 0.0f) return -ceil(-x);
        else return floorHelper1(x, 0.0f, ipow(2.0f, std::numeric_limits<float>::max_exponent - 1));
    }

    constexpr float ceilHelper2(float x, float guess, float dec) {
        if (guess - dec >= x) return ceilHelper2(x, guess - dec, dec);
        else if (dec <= 1.0f) return guess;
        else return ceilHelper2(x, guess, dec / 2.0f);
    }

    constexpr float ceilHelper1(float x, float guess, float dec) {
        if (dec < 8.0f) return ceilHelper2(x, guess, dec);
        else if (guess - dec >= x) return ceilHelper1(x, guess - dec, dec);
        else if (guess - (dec / 8.0f) * 7.0f >= x)
            return ceilHelper1(x, guess - (dec / 8.0f) * 7.0f, dec / 8.0f);
        else if (guess - (dec / 8.0f) * 6.0f >= x)
            return ceilHelper1(x, guess - (dec / 8.0f) * 6.0f, dec / 8.0f);
        else if (guess - (dec / 8.0f) * 5.0f >= x)
            return ceilHelper1(x, guess - (dec / 8.0f) * 5.0f, dec / 8.0f);
        else if (guess - (dec / 8.0f) * 4.0f >= x)
            return ceilHelper1(x, guess - (dec / 8.0f) * 4.0f, dec / 8.0f);
        else if (guess - (dec / 8.0f) * 3.0f >= x)
            return ceilHelper1(x, guess - (dec / 8.0f) * 3.0f, dec / 8.0f);
        else if (guess - (dec / 8.0f) * 2.0f >= x)
            return ceilHelper1(x, guess - (dec / 8.0f) * 2.0f, dec / 8.0f);
        else if (guess - dec / 8.0f >= x) return ceilHelper1(x, guess - dec / 8.0f, dec / 8.0f);
        else return ceilHelper1(x, guess, dec / 8.0f);
    }

    constexpr float ceil(float x) {
        if (x < 0.0f) return -floor(-x);
        else
            return ceilHelper1(x, ipow(2.0f, std::numeric_limits<float>::max_exponent - 1),
                               ipow(2.0f, std::numeric_limits<float>::max_exponent - 1));
    }

    constexpr float round(float x) { return x >= 0.0f ? floor(x + 0.5f) : ceil(x - 0.5f); }

    constexpr float remainder(float x, float y) { return x - y * round(x / y); }

    inline constexpr float test = floor(10.9f);
}

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
    static constexpr float squish(float rad) {
        if consteval {
            return RadianHelper::remainder(rad, 2.0f * Constants::PI);
        } else {
            return std::remainderf(rad, 2.0f * Constants::PI);
        }
    }
};
