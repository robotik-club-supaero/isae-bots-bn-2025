#include "geometry/Angle.hpp"
#include "geometry/Vector2D.hpp"

#include <cmath>
#include <numbers>

constexpr number_t Pi = Angle::Pi;

/// Take an angle in [-2*PI;2*PI] and return the equivalent angle in (-PI;PI]
/// The result is unspecified if `value` is not within [-2*PI;2*PI].
number_t modulo_x2(number_t value) {
    return (value > Pi ? value - 2 * Pi : (value <= -Pi ? value + 2 * Pi : value));
}

/// Take any angle and return the equivalent angle in (-PI;PI]
number_t modulo_pipi(number_t value) {
    return modulo_x2(fmod(value, 2 * Pi));
}

Angle::Angle(number_t value) : m_value(modulo_pipi(value)) {}

Angle::operator number_t() const {
    return m_value;
}
number_t Angle::value() const {
    return m_value;
}

Angle Angle::reverse() const {
    return *this + Angle(Pi);
}

Vector2D<Meter> Angle::toHeadingVector() const {
    return {std::cos(*this), std::sin(*this)};
}

Angle Angle::operator-() const {
    return Angle(-m_value);
}

void Angle::operator+=(Angle other) {
    // Cannot do m_value += other because condition `m_value in (-PI;PI]` would no longer hold
    *this = *this + other;
}
void Angle::operator-=(Angle other) {
    *this = *this - other;
}

Angle Angle::operator+(Angle other) const {
    return Angle(m_value + other.m_value);
}
Angle Angle::operator-(Angle other) const {
    return Angle(m_value - other.m_value);
}