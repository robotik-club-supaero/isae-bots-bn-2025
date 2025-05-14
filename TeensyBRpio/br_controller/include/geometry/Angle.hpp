#ifndef _ANGLE_HPP_
#define _ANGLE_HPP_

#include "defines/math.hpp"

template <typename Unit>
class Vector2D;

/**
 * A signed angle in radians, in (-PI, PI]. This type supports implicit conversion from and to double_t.
 */
class Angle {
  public:
    Angle(double_t value);

    operator double_t() const;
    double_t value() const;

    /// @return this + PI
    Angle reverse() const;

    /// Returns the heading vector of which this angle is the argument.
    Vector2D<Meter> toHeadingVector() const;

    Angle operator-() const;

    void operator+=(Angle other);
    void operator-=(Angle other);

    bool operator==(const Angle &other) const = default;
    Angle operator+(Angle other) const;
    Angle operator-(Angle other) const;

    static constexpr double_t Pi = std::numbers::pi_v<double_t>;

  private:
    double_t m_value;
};

namespace std {
inline double_t cos(Angle angle) {
    return cos(static_cast<double_t>(angle));
}
inline double_t sin(Angle angle) {
    return sin(static_cast<double_t>(angle));
}
} // namespace std

#endif