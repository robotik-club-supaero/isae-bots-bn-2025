#ifndef _ANGLE_HPP_
#define _ANGLE_HPP_

#include "defines/math.hpp"

template <typename Unit>
class Vector2D;

/**
 * A signed angle in radians, in (-PI, PI]. This type supports implicit conversion from and to number_t.
 */
class Angle {
  public:
    Angle(number_t value);

    operator number_t() const;
    number_t value() const;

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

    static constexpr number_t Pi = std::numbers::pi_v<number_t>;

  private:
    number_t m_value;
};

namespace std {
inline number_t cos(Angle angle) {
    return cos(static_cast<number_t>(angle));
}
inline number_t sin(Angle angle) {
    return sin(static_cast<number_t>(angle));
}
} // namespace std

#endif