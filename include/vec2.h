#pragma once
#include <cmath>
#include "raylib.h"

struct Vec2
{
    float x;
    float y;

    // --- Konstruktoren ---
    constexpr Vec2() : x(0.0f), y(0.0f) {}
    constexpr Vec2(float x_, float y_) : x(x_), y(y_) {}

    // automatische Umwandlung in raylib::Vector2
    operator Vector2() const { return { x, y }; }

    // --- Grund-Utilities ---
    inline float length()  const { return std::sqrt(x*x + y*y); }
    inline float lengthSq() const { return x*x + y*y; }

    inline float dot(const Vec2& rhs) const { return x*rhs.x + y*rhs.y; }

    inline float dist(const Vec2& rhs) const
    {
        const float dx = x - rhs.x;
        const float dy = y - rhs.y;
        return std::sqrt(dx*dx + dy*dy);
    }

    inline float distSq(const Vec2& rhs) const
    {
        const float dx = x - rhs.x;
        const float dy = y - rhs.y;
        return dx*dx + dy*dy;
    }

    inline bool isNearlyZero(float eps = 1e-6f) const
    {
        return std::fabs(x) < eps && std::fabs(y) < eps;
    }

    // Gibt eine normalisierte Kopie zurück (0-Vektor bleibt 0)
    inline Vec2 normalized(float eps = 1e-8f) const
    {
        const float len = length();
        return (len > eps) ? Vec2{x/len, y/len} : Vec2{0.0f, 0.0f};
    }

    // Normalisiert in-place (0-Vektor bleibt 0)
    inline void normalize(float eps = 1e-8f)
    {
        const float len = length();
        if (len > eps) { x /= len; y /= len; }
        else { x = 0.0f; y = 0.0f; }
    }

    // --- Operatoren (Vektor-Vektor) ---
    inline Vec2 operator+(const Vec2& rhs) const { return Vec2{x + rhs.x, y + rhs.y}; }
    inline Vec2 operator-(const Vec2& rhs) const { return Vec2{x - rhs.x, y - rhs.y}; }
    inline Vec2& operator+=(const Vec2& rhs) { x += rhs.x; y += rhs.y; return *this; }
    inline Vec2& operator-=(const Vec2& rhs) { x -= rhs.x; y -= rhs.y; return *this; }

    // --- Skalar-Operationen ---
    inline Vec2 operator*(float s) const { return Vec2{x * s, y * s}; }
    inline Vec2 operator/(float s) const { return Vec2{x / s, y / s}; }
    inline Vec2& operator*=(float s) { x *= s; y *= s; return *this; }
    inline Vec2& operator/=(float s) { x /= s; y /= s; return *this; }

    // Unary minus
    inline Vec2 operator-() const { return Vec2{-x, -y}; }

    // Vergleich (mit Toleranz)
    inline bool equals(const Vec2& rhs, float eps = 1e-6f) const
    {
        return std::fabs(x - rhs.x) < eps && std::fabs(y - rhs.y) < eps;
    }

    // Hilfen
    static inline Vec2 zero() { return Vec2{0.0f, 0.0f}; }
    static inline Vec2 one()  { return Vec2{1.0f, 1.0f}; }

    // Lerp (t in [0,1])
    static inline Vec2 lerp(const Vec2& a, const Vec2& b, float t)
    {
        return Vec2{ a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t };
    }
};

// Skalar * Vektor (damit auch "s * v" geht)
inline Vec2 operator*(float s, const Vec2& v) { return Vec2{s * v.x, s * v.y}; }
