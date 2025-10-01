#pragma once

#include <cmath>
#include <concepts>
#include <limits>

namespace VectorHelper {
    constexpr float sqrtNewtonRaphson(float x, float current, float previous) {
        if (current == previous) return current;
        else return sqrtNewtonRaphson(x, 0.5f * (current + (x / current)), current);
    }

    constexpr float constexprSqrt(float x) {
        if (x >= 0.0f && x < std::numeric_limits<float>::infinity())
            return sqrtNewtonRaphson(x, x, 0.0f);
    }
}

struct Vec2 {
    constexpr friend Vec2 operator+(Vec2 const& u, Vec2 const& v) {
        return { u.x + v.x, u.y + v.y };
    }
    constexpr friend Vec2 operator-(Vec2 const& u, Vec2 const& v) {
        return { u.x - v.x, u.y - v.y };
    }
    constexpr friend Vec2 operator*(Vec2 const& u, Vec2 const& v) {
        return { u.x * v.x, u.y * v.y };
    }
    constexpr friend Vec2 operator/(Vec2 const& u, Vec2 const& v) {
        return { u.x / v.x, u.y / v.y };
    }

    constexpr friend Vec2 operator*(float n, Vec2 const& u) { return { u.x * n, u.y * n }; }
    constexpr friend Vec2 operator*(Vec2 const& u, float n) { return { u.x * n, u.y * n }; }
    constexpr friend Vec2 operator/(Vec2 const& u, float n) { return { u.x / n, u.y / n }; }

    constexpr Vec2& operator+=(Vec2 const& u) {
        x += u.x;
        y += u.y;
        return *this;
    }
    constexpr Vec2& operator-=(Vec2 const& u) {
        x -= u.x;
        y -= u.y;
        return *this;
    }
    constexpr Vec2& operator*=(Vec2 u) {
        x *= u.x;
        y *= u.y;
        return *this;
    }
    constexpr Vec2& operator*=(float n) {
        x *= n;
        y *= n;
        return *this;
    }
    constexpr Vec2& operator/=(float n) {
        x /= n;
        y /= n;
        return *this;
    }

    static constexpr float dot(Vec2 const& u, Vec2 const& v) { return u.x * v.x + u.y * v.y; }

    template <typename F>
        requires requires(F f, float x) {
            { f(x) } -> std::same_as<float>;
        }
    static constexpr Vec2 transform(Vec2 const& u, F const& function) {
        return { function(u.x), function(u.y) };
    }
    template <typename F>
        requires requires(F f, float x) {
            { f(x) } -> std::same_as<float>;
        }
    constexpr Vec2& transform(F const& function) {
        x = function(x);
        y = function(y);
        return *this;
    }

    constexpr float lengthSquared() const { return x * x + y * y; }
    constexpr float length() const {
        if consteval {
            return VectorHelper::constexprSqrt(lengthSquared());
        } else {
            return std::sqrtf(lengthSquared());
        }
    }
    float angle() const { return std::atan2f(y, x); };

    static Vec2 fromPolar(float length, float angle) {
        return { length * std::cosf(angle), length * std::sinf(angle) };
    }

    float x{};
    float y{};
};

struct Vec3 {
    constexpr friend Vec3 operator+(Vec3 const& u, Vec3 const& v) {
        return { u.x + v.x, u.y + v.y, u.z + v.z };
    }
    constexpr friend Vec3 operator-(Vec3 const& u, Vec3 const& v) {
        return { u.x - v.x, u.y - v.y, u.z - v.z };
    }
    constexpr friend Vec3 operator*(Vec3 const& u, Vec3 const& v) {
        return { u.x * v.x, u.y * v.y, u.z * v.z };
    }
    constexpr friend Vec3 operator/(Vec3 const& u, Vec3 const& v) {
        return { u.x / v.x, u.y / v.y, u.z / v.z };
    }

    constexpr friend Vec3 operator*(float n, Vec3 const& u) {
        return { u.x * n, u.y * n, u.z * n };
    }
    constexpr friend Vec3 operator*(Vec3 const& u, float n) {
        return { u.x * n, u.y * n, u.z * n };
    }
    constexpr friend Vec3 operator/(Vec3 const& u, float n) {
        return { u.x / n, u.y / n, u.z / n };
    }

    constexpr Vec3& operator+=(Vec3 const& u) {
        x += u.x;
        y += u.y;
        z += u.z;
        return *this;
    }
    constexpr Vec3& operator-=(Vec3 const& u) {
        x -= u.x;
        y -= u.y;
        z -= u.z;
        return *this;
    }
    constexpr Vec3& operator*=(Vec3 u) {
        x *= u.x;
        y *= u.y;
        z *= u.z;
        return *this;
    }
    constexpr Vec3& operator*=(float n) {
        x *= n;
        y *= n;
        z *= n;
        return *this;
    }
    constexpr Vec3& operator/=(float n) {
        x /= n;
        y /= n;
        z /= n;
        return *this;
    }

    static constexpr float dot(Vec3 const& u, Vec3 const& v) {
        return u.x * v.x + u.y * v.y + u.z * v.z;
    }
    static constexpr Vec3 cross(Vec3 const& a, Vec3 const& b) {
        return { a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x };
    }

    template <typename F>
        requires requires(F f, float x) {
            { f(x) } -> std::same_as<float>;
        }
    static constexpr Vec3 transform(Vec3 const& u, F const& function) {
        return { function(u.x), function(u.y), function(u.z) };
    }
    template <typename F>
        requires requires(F f, float x) {
            { f(x) } -> std::same_as<float>;
        }
    constexpr Vec3& transform(F const& function) {
        x = function(x);
        y = function(y);
        z = function(z);
        return *this;
    }

    constexpr float lengthSquared() const { return x * x + y * y + z * z; }
    constexpr float length() const {
        if consteval {
            return VectorHelper::constexprSqrt(lengthSquared());
        } else {
            return std::sqrtf(lengthSquared());
        }
    }

    float x{};
    float y{};
    float z{};
};
