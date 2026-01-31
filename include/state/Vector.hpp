// https://www.elbeno.com/blog/?p=1211
// https://mazzo.li/posts/vectorized-atan2.html

#pragma once

#include <cmath>
#include <concepts>
#include <limits>
#include <numbers>
#include <utility>

namespace VectorDetail {
    constexpr float sqrtNewtonRaphson(float x, float current, float previous) {
        if (current == previous) return current;
        else return sqrtNewtonRaphson(x, 0.5f * (current + (x / current)), current);
    }

    constexpr float constexprSqrt(float x) {
        if (x >= 0.0f && x < std::numeric_limits<float>::infinity())
            return sqrtNewtonRaphson(x, x, 0.0f);
    }

    namespace Atan {
        constexpr float abs(float x) { return x > 0.0f ? x : -x; }

        constexpr float term(float x, int k) {
            return (2.0f * static_cast<float>(k) * x) /
                   ((2.0f * static_cast<float>(k) + 1.0f) * (1.0f + x));
        }

        constexpr float product(float x, int k) {
            return k == 1 ? term(x * x, k) : term(x * x, k) * product(x, k - 1);
        }

        constexpr float sum(float x, float currentSum, int n) {
            return currentSum + product(x, n) == currentSum
                       ? currentSum
                       : sum(x, currentSum + product(x, n), n + 1);
        }
    }

    constexpr float constexprAtan(float x) { return x / (1.0f + x * x) * Atan::sum(x, 1.0f, 1); }

    constexpr float constexprAtan2(float y, float x) {
        static constexpr float PI = std::numbers::pi_v<float>;

        if (x == 0.0f && y > 0.0f) return PI / 2.0f;
        if (x == 0.0f && y < 0.0f) return -PI / 2.0f;
        if (x == 0.0f && y == 0.0f) return 0.0f;

        bool const swap = Atan::abs(x) < Atan::abs(y);
        float const atanInput = (swap ? x : y) / (swap ? y : x);

        float result = constexprAtan(atanInput);
        if (swap) result = (atanInput >= 0.0f ? PI / 2.0f : -PI / 2.0f) - result;

        if (x >= 0.0f) return result;
        if (x < 0.0f && y >= 0.0f) return result + PI;
        if (x < 0.0f && y < 0.0f) return result - PI;
    }
}

template <typename F, typename... Vectors>
concept TransformFunction = requires(F f, Vectors... vectors) {
    { f((vectors, std::declval<float>())...) } -> std::same_as<float>;
};

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
    static constexpr float cross(Vec2 const& u, Vec2 const& v) { return u.x * v.y - u.y * v.x; }

    template <typename F, std::same_as<Vec2>... Vectors>
        requires TransformFunction<F, Vectors...>
    static constexpr Vec2 transform(F const& function, Vectors const&... vectors) {
        return { function(vectors.x...), function(vectors.y...) };
    }
    template <typename F>
        requires TransformFunction<F, Vec2>
    constexpr Vec2& transform(F const& function) {
        x = function(x);
        y = function(y);
        return *this;
    }

    constexpr float lengthSquared() const { return x * x + y * y; }
    constexpr float length() const {
        if consteval {
            return VectorDetail::constexprSqrt(lengthSquared());
        } else {
            return std::sqrtf(lengthSquared());
        }
    }
    constexpr float angle() const {
        if consteval {
            return VectorDetail::constexprAtan2(y, x);
        } else {
            return std::atan2f(y, x);
        }
    }

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

    template <typename F, std::same_as<Vec3>... Vectors>
        requires TransformFunction<F, Vectors...>
    static constexpr Vec3 transform(F const& function, Vectors const&... vectors) {
        return { function(vectors.x...), function(vectors.y...), function(vectors.z...) };
    }
    template <typename F>
        requires TransformFunction<F, Vec3>
    constexpr Vec3& transform(F const& function) {
        x = function(x);
        y = function(y);
        z = function(z);
        return *this;
    }

    constexpr float lengthSquared() const { return x * x + y * y + z * z; }
    constexpr float length() const {
        if consteval {
            return VectorDetail::constexprSqrt(lengthSquared());
        } else {
            return std::sqrtf(lengthSquared());
        }
    }

    float x{};
    float y{};
    float z{};
};
